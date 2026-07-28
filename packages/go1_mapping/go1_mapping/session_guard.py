#!/usr/bin/env python3
"""Mapping session health guard."""
from __future__ import annotations
from collections import deque
import math
from pathlib import Path
import shutil
import threading
import time
from typing import Any, Callable, Mapping
from go1_mapping.manifest import write_manifest_atomic

WINDOW_SECONDS=10.0

def _quaternion(x,y,z,w):
 values=tuple(map(float,(x,y,z,w)))
 if not all(math.isfinite(v) for v in values): raise ValueError('quaternion must be finite')
 norm=math.sqrt(sum(v*v for v in values))
 if norm==0: raise ValueError('quaternion must be nonzero')
 return tuple(v/norm for v in values)
def quaternion_yaw(x,y,z,w):
 x,y,z,w=_quaternion(x,y,z,w);return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def wrap_angle(angle): return math.atan2(math.sin(angle),math.cos(angle))

class HealthWindow:
 def __init__(self,required_hz:Mapping[str,float],max_gap_sec:float,initialization_sec:float,started_at:float):
  self.required_hz={k:float(v) for k,v in required_hz.items()};self.max_gap_sec=float(max_gap_sec);self.initialization_sec=float(initialization_sec);self.started_at=float(started_at);self.receipts={k:deque() for k in self.required_hz};self.headers={};self.source_seen={k:False for k in self.required_hz};self.source_invalid={k:False for k in self.required_hz};self.regressions={k:deque() for k in self.required_hz};self.lock=threading.RLock()
 def observe(self,topic,stamp_sec,sensor_stamp_sec=None):
  with self.lock:
   if topic not in self.receipts: raise ValueError(f'unknown health topic: {topic}')
   receipt=float(stamp_sec);q=self.receipts[topic]
   if q and receipt<q[-1]: self.regressions[topic].append((receipt,'receipt',q[-1],receipt))
   else: q.append(receipt)
   if sensor_stamp_sec is None:
    if self.source_seen[topic]: self.source_invalid[topic]=True
   else:
    header=float(sensor_stamp_sec);previous=self.headers.get(topic)
    if previous is not None and header<previous:self.regressions[topic].append((receipt,'sensor',previous,header))
    if previous is None or header>previous:self.source_invalid[topic]=False
    self.source_seen[topic]=True;self.headers[topic]=header if previous is None else max(previous,header)
 def measurements(self,now_sec):
  with self.lock:
   now=float(now_sec);cutoff=now-WINDOW_SECONDS;rates={};gaps={}
   for topic,q in self.receipts.items():
    while q and q[0]<cutoff:q.popleft()
    rates[topic]=(len(q)-1)/(q[-1]-q[0]) if len(q)>1 and q[-1]>q[0] else 0.0
    gaps[topic]=max(0.0,now-q[-1]) if q else max(0.0,now-self.started_at)
   for q in self.regressions.values():
    while q and q[0][0]<cutoff:q.popleft()
   return {'rates_hz':rates,'max_gaps_sec':gaps}
 def evaluate(self,now_sec):
  with self.lock:
   m=self.measurements(now_sec)
   if now_sec<self.started_at+self.initialization_sec:return []
   result=[]
   for topic,required in self.required_hz.items():
    rate,gap=m['rates_hz'][topic],m['max_gaps_sec'][topic]
    if rate<required:result.append(f'{topic} rate {rate:.2f} Hz below required {required:.2f} Hz')
    if gap>self.max_gap_sec:result.append(f'{topic} gap {gap:.2f}s exceeds {self.max_gap_sec:.2f}s')
    result.extend(f'{topic} {kind} timestamp regression: {current:.9f} < {previous:.9f}' for _,kind,previous,current in self.regressions[topic])
    if self.source_invalid[topic]:result.append(f'{topic} source timestamp reset/invalid')
   return result

def free_gib(path): return shutil.disk_usage(path).free/2**30
def should_abort_disk(free,abort=50.): return float(free)<float(abort)
def pose_return_error(first,latest):
 if first is None or latest is None:return None
 def unpack(p):
  q=p['orientation'];v=p['position'];return float(v['x']),float(v['y']),quaternion_yaw(q.get('x',0),q.get('y',0),q['z'],q['w'])
 fx,fy,fa=unpack(first);lx,ly,la=unpack(latest);return {'xy_m':math.hypot(lx-fx,ly-fy),'yaw_rad':abs(wrap_angle(la-fa))}
def evaluate_and_record(*,session_dir,window,now_sec,free_gib_value=None,abort_free_gib=50.,first_stable_pose=None,latest_pose=None,writer:Callable=write_manifest_atomic):
 root=Path(session_dir);validation=root/'validation';validation.mkdir(parents=True,exist_ok=True);remaining=free_gib(root) if free_gib_value is None else float(free_gib_value);reasons=window.evaluate(now_sec)
 if should_abort_disk(remaining,abort_free_gib):reasons.append(f'free disk {remaining:.2f} GiB below abort limit {float(abort_free_gib):.2f} GiB')
 payload={'timestamp_sec':float(now_sec),'free_gib':remaining,'abort_free_gib':float(abort_free_gib),'first_stable_odometry_pose':first_stable_pose,'latest_odometry_pose':latest_pose,'return_error':pose_return_error(first_stable_pose,latest_pose),'reasons':reasons,'write_errors':[]};payload.update(window.measurements(now_sec))
 def persist(target):
  try:writer(target,payload)
  except Exception as error:
   message=f'{target.name} write failed: {error}';payload['write_errors'].append(message);payload['reasons'].append(message)
 persist(validation/'health.yaml')
 if payload['reasons']:persist(validation/'guard_failure.yaml');return 2,payload
 return 0,payload

def _source_stamp(message):
 timebase=getattr(message,'timebase',0)
 if timebase:return float(timebase)/1e9
 stamp=getattr(getattr(message,'header',None),'stamp',None)
 if stamp is None or (stamp.sec==0 and stamp.nanosec==0):return None
 return float(stamp.sec)+float(stamp.nanosec)/1e9
def _pose(message):
 p=message.position;q=message.orientation
 try:x,y,z,w=_quaternion(q.x,q.y,q.z,q.w)
 except ValueError:return None
 if not all(math.isfinite(v) for v in (p.x,p.y)):return None
 return {'position':{'x':float(p.x),'y':float(p.y)},'orientation':{'x':x,'y':y,'z':z,'w':w}}
class SessionGuardNode:
 def __init__(self):
  import rclpy
  from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
  from rclpy.node import Node
  from rclpy.qos import QoSProfile,HistoryPolicy,ReliabilityPolicy,DurabilityPolicy
  from sensor_msgs.msg import Imu
  from nav_msgs.msg import Odometry
  from livox_ros_driver2.msg import CustomMsg
  self.node=Node('mapping_session_guard');self.rclpy=rclpy
  for name,value in [('session_dir',''),('abort_free_gib',50.),('lidar_min_hz',8.),('imu_min_hz',100.),('odom_min_hz',8.),('max_gap_sec',1.),('initialization_sec',15.)]:self.node.declare_parameter(name,value)
  directory=self.node.get_parameter('session_dir').value
  if not directory:raise ValueError('session_dir parameter is required')
  self.session_dir=Path(directory);self.abort=float(self.node.get_parameter('abort_free_gib').value);self.initialization=float(self.node.get_parameter('initialization_sec').value);self.started=time.monotonic();self.window=HealthWindow({k:self.node.get_parameter(k+'_min_hz').value for k in ('lidar','imu','odom')},self.node.get_parameter('max_gap_sec').value,self.initialization,self.started);self.first_pose=self.latest_pose=None;self.exit_code=0
  def qos(depth):return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=depth,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
  self.callback_groups=[MutuallyExclusiveCallbackGroup() for _ in range(4)]
  self.node.create_subscription(CustomMsg,'/livox/lidar',self.lidar,qos(20),callback_group=self.callback_groups[0]);self.node.create_subscription(Imu,'/livox/imu',self.imu,qos(500),callback_group=self.callback_groups[1]);self.node.create_subscription(Odometry,'/Odometry',self.odom,qos(50),callback_group=self.callback_groups[2]);self.node.create_timer(1.,self.timer,callback_group=self.callback_groups[3])
 def now(self):return time.monotonic()
 def lidar(self,msg):self.window.observe('lidar',self.now(),_source_stamp(msg))
 def imu(self,msg):self.window.observe('imu',self.now(),_source_stamp(msg))
 def odom(self,msg):
  now=self.now();self.window.observe('odom',now,_source_stamp(msg));pose=_pose(msg.pose.pose)
  if pose is not None:self.latest_pose=pose;self.first_pose=pose if self.first_pose is None and now>=self.started+self.initialization else self.first_pose
 def timer(self):
  try:code,payload=evaluate_and_record(session_dir=self.session_dir,window=self.window,now_sec=self.now(),abort_free_gib=self.abort,first_stable_pose=self.first_pose,latest_pose=self.latest_pose)
  except Exception as error:code,payload=2,{'reasons':[f'health guard internal failure: {error}']}
  if code==2:self.exit_code=2;self.node.get_logger().error('; '.join(payload['reasons']))
 def destroy_node(self):return self.node.destroy_node()
def main(args=None,rclpy_module=None,guard_factory=None):
 if rclpy_module is None:
  import rclpy as rclpy_module
 if guard_factory is None:guard_factory=SessionGuardNode
 rclpy_module.init(args=args);guard=None;executor=None
 try:
  guard=guard_factory()
  if hasattr(rclpy_module,'executors'):
   executor=rclpy_module.executors.MultiThreadedExecutor(num_threads=4);executor.add_node(guard.node)
  while rclpy_module.ok() and guard.exit_code==0:
   if executor is None:rclpy_module.spin_once(guard.node,timeout_sec=0.5)
   else:executor.spin_once(timeout_sec=0.5)
  return guard.exit_code
 except KeyboardInterrupt:
  return 0
 finally:
  if executor is not None:
   try:executor.remove_node(guard.node);executor.shutdown()
   except KeyboardInterrupt:pass
  try:
   if guard is not None:guard.destroy_node()
  except KeyboardInterrupt:pass
  try:
   if rclpy_module.ok():rclpy_module.shutdown()
  except KeyboardInterrupt:pass
if __name__=='__main__':raise SystemExit(main())
