![SubTCA.png](https://github.com/osrf/subt/wiki/images/SubTCA.png)

## Configuration Overview
[AWS EC2 g3.4xlarge](https://aws.amazon.com/ec2/instance-types/g3/) instances are used for running simulation and for each robot solution. A `g3.4xlarge` instance has 1 GPU with 8 GiB of GPU memory, and 16 vCPUs with 122 GIB of CPU memory. The Amazon Machine Image (AMI) deployed on these instances supports CUDA 10 with nvidia-docker2. Communication between EC2 instances relies on [Ignition Transport](https://ignitionrobotics.org/features) and the ROS communication is isolated to each robot EC2 instance, as shown above.

There are two docker containers running on each robot EC2 instance. The Bridge Container; which cannot be modified by a competitor, is responsible for starting the ROS master and ROS1 Ignition Transport bridge. The Solution Container consists of a competitor’s control code for the robot. Competitors can submit one docker image per robot; where each image can be different or identical. 

The results of inspecting `/proc/sys/kernel` and running `ulimit` can be found in this list.

<details>

### Listing of values from /proc/sys/kernel
```
acct: 4	2	30
acpi_video_flags: 0
auto_msgmni: 0
bootloader_type: 114
bootloader_version: 2
cad_pid: 1
cap_last_cap: 37
core_pattern: core
core_pipe_limit: 0
core_uses_pid: 1
ctrl-alt-del: 0
dmesg_restrict: 0
domainname: (none)
ftrace_dump_on_oops: 0
ftrace_enabled: 1
hardlockup_all_cpu_backtrace: 0
hardlockup_panic: 0
hostname: ip-172-24-31-57.us-west-2.compute.internal
hotplug: /sbin/hotplug
hung_task_check_count: 4194304
hung_task_panic: 0
hung_task_timeout_secs: 120
hung_task_warnings: 10
io_delay_type: 0
kexec_load_disabled: 0
keys:
    gc_delay: 300
    maxbytes: 20000
    maxkeys: 200
    persistent_keyring_expiry: 259200
    root_maxbytes: 25000000
    root_maxkeys: 1000000
kptr_restrict: 1
latencytop: 0
max_lock_depth: 1024
modprobe: /sbin/modprobe
modules_disabled: 0
msg_next_id: -1
msgmax: 8192
msgmnb: 16384
msgmni: 32000
ngroups_max: 65536
nmi_watchdog: 0
ns_last_pid: 108199
numa_balancing: 0
numa_balancing_scan_delay_ms: 1000
numa_balancing_scan_period_max_ms: 60000
numa_balancing_scan_period_min_ms: 1000
numa_balancing_scan_size_mb: 256
osrelease: 4.14.232-177.418.amzn2.x86_64
ostype: Linux
overflowgid: 65534
overflowuid: 65534
panic: 10
panic_on_io_nmi: 0
panic_on_oops: 1
panic_on_rcu_stall: 0
panic_on_stackoverflow: 0
panic_on_unrecovered_nmi: 0
panic_on_warn: 0
perf_cpu_time_max_percent: 25
perf_event_max_contexts_per_stack: 8
perf_event_max_sample_rate: 100000
perf_event_max_stack: 127
perf_event_mlock_kb: 516
perf_event_paranoid: 2
pid_max: 131072
poweroff_cmd: /sbin/poweroff
print-fatal-signals: 0
printk: 8	4	1	7
printk_delay: 0
printk_devkmsg: ratelimit
printk_ratelimit: 5
printk_ratelimit_burst: 10
pty:
    max: 4096
    nr: 2
    reserve: 1024
random:
    boot_id: de05c82c-685c-44cd-be5d-41ee7b76475e
    entropy_avail: 3772
    poolsize: 4096
    read_wakeup_threshold: 64
    urandom_min_reseed_secs: 60
    uuid: 6c0289bd-e1af-4e5a-8e2e-ccb61474e311
    write_wakeup_threshold: 3072
randomize_va_space: 2
real-root-dev: 0
sched_autogroup_enabled: 0
sched_cfs_bandwidth_slice_us: 5000
sched_child_runs_first: 0
sched_latency_ns: 24000000
sched_migration_cost_ns: 500000
sched_min_granularity_ns: 3000000
sched_nr_migrate: 32
sched_rr_timeslice_ms: 100
sched_rt_period_us: 1000000
sched_rt_runtime_us: 950000
sched_schedstats: 0
sched_time_avg_ms: 1000
sched_tunable_scaling: 1
sched_wakeup_granularity_ns: 4000000
seccomp:
    actions_avail: kill_process kill_thread trap errno trace log allow
    actions_logged: kill_process kill_thread trap errno trace log
sem: 32000	1024000000	500	32000
sem_next_id: -1
shm_next_id: -1
shm_rmid_forced: 0
shmall: 18446744073692774399
shmmax: 18446744073692774399
shmmni: 4096
soft_watchdog: 1
softlockup_all_cpu_backtrace: 0
softlockup_panic: 0
stack_tracer_enabled: 0
sysctl_writes_strict: 1
sysrq: 16
tainted: 12289
threads-max: 982770
timer_migration: 1
traceoff_on_warning: 0
tracepoint_printk: 0
unknown_nmi_panic: 0
unprivileged_bpf_disabled: 0
version: #1 SMP Tue Jun 15 20:57:50 UTC 2021
watchdog: 1
watchdog_cpumask: 0-127
watchdog_thresh: 10
```

### ulimit -a

```
core file size          (blocks, -c) 0
data seg size           (kbytes, -d) unlimited
scheduling priority             (-e) 0
file size               (blocks, -f) unlimited
pending signals                 (-i) 30446
max locked memory       (kbytes, -l) unlimited
max memory size         (kbytes, -m) unlimited
open files                      (-n) 65535
pipe size            (512 bytes, -p) 8
POSIX message queues     (bytes, -q) 819200
real-time priority              (-r) 0
stack size              (kbytes, -s) 10240
cpu time               (seconds, -t) unlimited
max user processes              (-u) unlimited
virtual memory          (kbytes, -v) unlimited
file locks                      (-x) unlimited
```
</details>

## Network Isolation
Each Robot EC2 instance runs a firewall on the host; which blocks all network traffic except for connections to the Bridge Container. This prevents the Solution Container from communicating directly with other robots and the outside world. In addition, a pod-level firewall is put in place to ensure that communication can only happen between solution pods and their respective bridges.

Ignition Transport is be used between the Bridge Container and Simulation Container. This allows each robot to run a ROS master, thereby avoiding a ROS multi-master setup and the need to block specific ROS topics. Filtering of Ignition Transport topics is performed by the ROS1 Ignition bridge. The filtering will expose only the topics needed by the robot associated with the Robot EC2 instance.

## Logging

Gazebo state information and server console messages are logged on the Ignition Gazebo instance. ROS console messages are captured in the Bridge Container on each Robot instance.

## Websocket Connections

The simulation server contains a websocket server that can forward topic information using the websocket protocol. The websocket server is available publicly and clients are free to connect at any time. However, established connections need to perform an authentication process before being able to receive simulation data. For information about the websocket server, refer to the [Websocket Server documentation](https://github.com/ignitionrobotics/ign-launch/blob/ign-launch4/plugins/websocket_server/WebsocketServer.hh#L35).

## Mapping Server

A mapping server is launched to perform simulation processing and aid DARPA in the evaluation of submissions. The mapping server is launched in a separate [`AWS EC2 c5.4xlarge`](https://aws.amazon.com/ec2/instance-types/c5/) instance, and cannot be accessed directly by team solutions. Instead, team solutions can interact with the mapping server by publishing to specific topics to have their data processed. For more information on how to interact with the mapping server, refer to the [Mapping Server section of the API article](https://github.com/osrf/subt/wiki/api#mapping-server).

**[Next, running Cloudsim locally](https://github.com/osrf/subt/wiki/Cloudsim%20Docker%20Compose)**
