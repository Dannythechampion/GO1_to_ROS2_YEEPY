from __future__ import annotations
from collections import deque
import math, shutil
from pathlib import Path
from typing import Any, Mapping
from go1_mapping.manifest import write_manifest_atomic

class HealthWindow:
 def __init__(self,required_hz:Mapping[str,float],max_gap_sec:float,initialization_sec:float,started_at:float):
  self.required_hz={k:float(v) for k,v in required_hz.items()};self.max_gap_sec=float(max_gap_sec);self.initialization_sec=float(initialization_sec);self.started_at=float(started_at);self.r={k:deque() for k in self.required_hz};self.h={};self.x={k:deque() for k in self.required_hz}
 def observe(self,topic:str,stamp_sec:float,sensor_stamp_sec:float|None=None)->None:
  if topic not in self.r: raise ValueError(f'unknown health topic: {topic}')
  t=float(stamp_sec);self.r[topic].append(t)
  if sensor_stamp_sec is not None:
   h=float(sensor_stamp_sec);p=self.h.get(topic)
   if p is not None and h<p:self.x[topic].append((t,p,h))
   self.h[topic]=h if p is None else max(p,h)
 def measurements(self,now_sec:float)->dict[str,dict[str,float]]:
  now=float(now_sec);cut=now-10.;rates={};gaps={}
  for k,q in self.r.items():
   while q and q[0]<cut:q.popleft()
   rates[k]=(len(q)-1)/(q[-1]-q[0]) if len(q)>1 and q[-1]>q[0] else 0.
   gaps[k]=now-q[-1] if q else now-self.started_at
  for q in self.x.values():
   while q and q[0][0]<cut:q.popleft()
  return {'rates_hz':rates,'max_gaps_sec':gaps}
 def evaluate(self,now_sec:float)->list[str]:
  m=self.measurements(now_sec)
  if now_sec<self.started_at+self.initialization_sec:return []
  e=[]
  for k,need in self.required_hz.items():
   if m['rates_hz'][k]<need:e.append(f'{k} rate {m["rates_hz"][k]:.2f} Hz below required {need:.2f} Hz')
   if m['max_gaps_sec'][k]>self.max_gap_sec:e.append(f'{k} gap {m["max_gaps_sec"][k]:.2f}s exceeds {self.max_gap_sec:.2f}s')
   e.extend(f'{k} sensor timestamp regression: {c:.9f} < {p:.9f}' for _,p,c in self.x[k])
  return e

def free_gib(session_dir:str|Path)->float:return shutil.disk_usage(session_dir).free/2**30
def should_abort_disk(free_gib_value:float,abort_free_gib:float=50.)->bool:return float(free_gib_value)<float(abort_free_gib)
def quaternion_yaw(x:float,y:float,z:float,w:float)->float:return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def wrap_angle(angle:float)->float:return math.atan2(math.sin(angle),math.cos(angle))
def pose_return_error(a:Mapping[str,Any]|None,b:Mapping[str,Any]|None)->dict[str,float]|None:
 if a is None or b is None:return None
 def c(p):
  q=p['orientation'];v=p['position'];return float(v['x']),float(v['y']),quaternion_yaw(float(q.get('x',0)),float(q.get('y',0)),float(q['z']),float(q['w']))
 ax,ay,aa=c(a);bx,by,ba=c(b);return {'xy_m':math.hypot(bx-ax,by-ay),'yaw_rad':abs(wrap_angle(ba-aa))}
def evaluate_and_record(*,session_dir:str|Path,window:HealthWindow,now_sec:float,free_gib_value:float|None=None,abort_free_gib:float=50.,first_stable_pose=None,latest_pose=None):
 root=Path(session_dir);d=root/'validation';d.mkdir(parents=True,exist_ok=True);free=free_gib(root) if free_gib_value is None else float(free_gib_value);reasons=window.evaluate(now_sec)
 if should_abort_disk(free,abort_free_gib):reasons.append(f'free disk {free:.2f} GiB below abort limit {float(abort_free_gib):.2f} GiB')
 p={'timestamp_sec':float(now_sec),'free_gib':free,'abort_free_gib':float(abort_free_gib),'first_stable_odometry_pose':first_stable_pose,'latest_odometry_pose':latest_pose,'return_error':pose_return_error(first_stable_pose,latest_pose),'reasons':reasons};p.update(window.measurements(now_sec));write_manifest_atomic(d/'health.yaml',p)
 if reasons:write_manifest_atomic(d/'guard_failure.yaml',p);return 2,p
 return 0,p

def _stamp(msg):
 s=getattr(getattr(msg,'header',None),'stamp',None);return None if s is None else float(s.sec)+float(s.nanosec)/1e9
class SessionGuardNode:
 def __init__(self):
  import rclpy
  from rclpy.node import Node
  from rclpy.qos import SensorDataQoS
  from sensor_msgs.msg import Imu
  from nav_msgs.msg import Odometry
  from livox_ros_driver2.msg import CustomMsg
  self.n=Node('mapping_session_guard');self.rc=rclpy
  for k,v in [('session_dir',''),('abort_free_gib',50.),('lidar_min_hz',8.),('imu_min_hz',100.),('odom_min_hz',8.),('max_gap_sec',1.),('initialization_sec',15.)]:self.n.declare_parameter(k,v)
  sd=self.n.get_parameter('session_dir').value
  if not sd:raise ValueError('session_dir parameter is required')
  self.sd=Path(sd);self.abort=float(self.n.get_parameter('abort_free_gib').value);self.init=float(self.n.get_parameter('initialization_sec').value);self.started=self.now();self.w=HealthWindow({k:self.n.get_parameter(k+'_min_hz').value for k in ('lidar','imu','odom')},self.n.get_parameter('max_gap_sec').value,self.init,self.started);self.first=self.latest=None;self.exit_code=0;q=SensorDataQoS();self.n.create_subscription(CustomMsg,'/livox/lidar',self.lidar,q);self.n.create_subscription(Imu,'/livox/imu',self.imu,q);self.n.create_subscription(Odometry,'/Odometry',self.odom,q);self.n.create_timer(1.,self.timer)
 def now(self):return self.n.get_clock().now().nanoseconds/1e9
 def lidar(self,m):self.w.observe('lidar',self.now(),_stamp(m))
 def imu(self,m):self.w.observe('imu',self.now(),_stamp(m))
 def odom(self,m):
  now=self.now();self.w.observe('odom',now,_stamp(m));p=m.pose.pose.position;q=m.pose.pose.orientation;values=(p.x,p.y,q.x,q.y,q.z,q.w)
  if all(math.isfinite(value) for value in values) and not math.isclose(sum(value*value for value in values[2:]),0):
   self.latest={'position':{'x':float(p.x),'y':float(p.y)},'orientation':{'x':float(q.x),'y':float(q.y),'z':float(q.z),'w':float(q.w)}}
   if self.first is None and now>=self.started+self.init:self.first=self.latest
 def timer(self):
  code,p=evaluate_and_record(session_dir=self.sd,window=self.w,now_sec=self.now(),abort_free_gib=self.abort,first_stable_pose=self.first,latest_pose=self.latest)
  if code==2:self.exit_code=2;self.rc.shutdown()
 def destroy_node(self):return self.n.destroy_node()
def main(args=None):
 import rclpy
 rclpy.init(args=args);node=None
 try:node=SessionGuardNode();rclpy.spin(node.n);return node.exit_code
 finally:
  if node:node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())