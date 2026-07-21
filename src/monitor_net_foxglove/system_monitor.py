#!/usr/bin/env python3
"""
system_monitor.py
CPU / RAM / GPU 사용률을 Float32 토픽으로 발행한다.

GPU 읽기 방식은 3단계 폴백:
  1) pynvml (nvidia-ml-py) 이 있으면 사용
  2) 없거나 실패하면 nvidia-smi 파싱 (드라이버와 함께 항상 설치됨)
  3) 둘 다 실패하면 CPU/RAM만 발행

발행 토픽 (<host>는 노트북 hostname):
  /monitor/<host>/cpu_percent       (Float32, %)
  /monitor/<host>/ram_percent       (Float32, %)
  /monitor/<host>/gpu_percent       (Float32, %)      GPU 있을 때
  /monitor/<host>/gpu_mem_percent   (Float32, %)      GPU 있을 때
  /monitor/<host>/gpu_temp_c        (Float32, C)      GPU 있을 때

실행:
  python3 system_monitor.py
"""
import socket
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import psutil

# --- GPU 읽기 백엔드 결정 ---
_GPU_MODE = None  # 'nvml' | 'smi' | None
try:
    import pynvml
    pynvml.nvmlInit()
    _GPU_HANDLE = pynvml.nvmlDeviceGetHandleByIndex(0)
    _GPU_MODE = 'nvml'
except Exception:
    # pynvml 실패 → nvidia-smi 사용 가능한지 확인
    try:
        subprocess.check_output(['nvidia-smi', '-L'], encoding='utf-8', timeout=3.0)
        _GPU_MODE = 'smi'
    except Exception:
        _GPU_MODE = None


def read_gpu():
    """(gpu_pct, gpu_mem_pct, temp_c) 반환. 실패 시 None."""
    if _GPU_MODE == 'nvml':
        util = pynvml.nvmlDeviceGetUtilizationRates(_GPU_HANDLE)
        mem = pynvml.nvmlDeviceGetMemoryInfo(_GPU_HANDLE)
        temp = pynvml.nvmlDeviceGetTemperature(_GPU_HANDLE, pynvml.NVML_TEMPERATURE_GPU)
        return float(util.gpu), float(mem.used) / float(mem.total) * 100.0, float(temp)
    if _GPU_MODE == 'smi':
        out = subprocess.check_output(
            ['nvidia-smi',
             '--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu',
             '--format=csv,noheader,nounits'],
            encoding='utf-8', timeout=2.0)
        line = out.strip().splitlines()[0]
        util, mem_used, mem_total, temp = [float(x) for x in line.split(',')]
        return util, mem_used / mem_total * 100.0, temp
    return None


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        host = socket.gethostname().replace('-', '_')
        self.pub_cpu = self.create_publisher(Float32, f'/monitor/{host}/cpu_percent', 10)
        self.pub_ram = self.create_publisher(Float32, f'/monitor/{host}/ram_percent', 10)

        self.gpu = _GPU_MODE is not None
        if self.gpu:
            self.pub_gpu = self.create_publisher(Float32, f'/monitor/{host}/gpu_percent', 10)
            self.pub_gmem = self.create_publisher(Float32, f'/monitor/{host}/gpu_mem_percent', 10)
            self.pub_gtemp = self.create_publisher(Float32, f'/monitor/{host}/gpu_temp_c', 10)
            self.get_logger().info(f"GPU metrics via '{_GPU_MODE}'")
        else:
            self.get_logger().warn("GPU not available - CPU/RAM only")

        psutil.cpu_percent(interval=None)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(f"system_monitor started on host '{host}'")

    def tick(self):
        cpu = psutil.cpu_percent(interval=None)
        ram = psutil.virtual_memory().percent
        self.pub_cpu.publish(Float32(data=float(cpu)))
        self.pub_ram.publish(Float32(data=float(ram)))
        line = f"CPU {cpu:5.1f}%   RAM {ram:5.1f}%"

        if self.gpu:
            try:
                g = read_gpu()
                if g is not None:
                    gpu_pct, gmem_pct, temp = g
                    self.pub_gpu.publish(Float32(data=gpu_pct))
                    self.pub_gmem.publish(Float32(data=gmem_pct))
                    self.pub_gtemp.publish(Float32(data=temp))
                    line += f"   GPU {gpu_pct:5.1f}%   GMEM {gmem_pct:5.1f}%   {temp:.0f}C"
            except Exception as e:
                line += f"   [GPU read error: {e}]"

        self.get_logger().info(line)


def main():
    rclpy.init()
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()