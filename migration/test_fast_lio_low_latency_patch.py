import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).with_name("patch_fast_lio_low_latency.py")

SOURCE = r'''
mutex mtx_buffer;
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool    is_first_lidar = true;
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if(is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;
    }
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

class LaserMappingNode {
public:
    LaserMappingNode()
    {
        sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));
    }

private:
    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                flg_first_scan = false;
                return;
            }
            double t0 = omp_get_wtime();
            if (feats_undistort->empty())
            {
                return;
            }
            double t5 = omp_get_wtime();
        }
    }

    void map_publish_callback()
    {
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
};
'''


def write_fast_lio_fixture(tmp_path: Path, source: str = SOURCE) -> Path:
    path = tmp_path / "laserMapping.cpp"
    path.write_text(source, encoding="utf-8")
    return path


def run_patch(path: Path, mode: str = "diagnostic") -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), "--mode", mode, str(path)],
        capture_output=True,
        text=True,
        check=False,
    )


def test_diagnostic_mode_instruments_internal_queue_without_dropping_normal_scans(tmp_path):
    source = write_fast_lio_fixture(tmp_path)

    result = run_patch(source)

    assert result.returncode == 0, result.stderr
    patched = source.read_text(encoding="utf-8")
    assert "FAST_LIO_REALTIME_DIAGNOSTICS" in patched
    assert "[fast_lio_realtime]" in patched
    assert "queue_depth=%zu" in patched
    assert "front_age=%.6f" in patched
    assert "process_ms=%.3f" in patched
    assert "imu_margin=%.6f" in patched
    assert "realtime_diagnostics_timer_" in patched
    assert "std::lock_guard<std::mutex> lock(mtx_buffer);" in patched
    assert "rclcpp::SensorDataQoS().keep_last(1)" in patched
    assert patched.count("lidar_buffer.push_back(ptr);") == 2
    assert "FAST_LIO_BOUNDED_BUFFER" not in patched


def test_diagnostic_mode_repairs_paired_queue_reset_on_both_lidar_callbacks(tmp_path):
    source = write_fast_lio_fixture(tmp_path)

    result = run_patch(source)

    assert result.returncode == 0, result.stderr
    patched = source.read_text(encoding="utf-8")
    assert patched.count("clear_lidar_buffers_locked();") == 2
    helper = patched.split("void clear_lidar_buffers_locked()", 1)[1].split("}", 1)[0]
    assert "lidar_buffer.clear();" in helper
    assert "time_buffer.clear();" in helper
    assert "lidar_pushed = false;" in helper


def test_diagnostic_patch_is_idempotent(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    first = run_patch(source)
    assert first.returncode == 0, first.stderr
    once = source.read_bytes()

    second = run_patch(source)

    assert second.returncode == 0, second.stderr
    assert source.read_bytes() == once


def test_patch_rejects_unknown_upstream_without_partial_write(tmp_path):
    source = write_fast_lio_fixture(tmp_path, "unrecognized source\n")
    before = source.read_bytes()

    result = run_patch(source)

    assert result.returncode != 0
    assert "expected exactly one" in result.stderr
    assert source.read_bytes() == before
