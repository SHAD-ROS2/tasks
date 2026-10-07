"""Bounded laboratory commands; JSON is evidence, not an authoritative grade."""
import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback
from ament_index_python.packages import get_package_share_directory
from .configuration import load_settings, validate_settings, names
from .processes import cleanup_session


def arguments():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['observe','scenario','check','control'])
    parser.add_argument('operation',nargs='?',choices=['pause','play','step','reset'])
    parser.add_argument('--mode',choices=['practice','homework'],default='practice')
    parser.add_argument('--namespace',default='rover')
    parser.add_argument('--frame-prefix',default='rover/')
    parser.add_argument('--world-name',default='lab')
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--target-rtf',type=float,default=1.0)
    parser.add_argument('--profile',choices=['ideal','white','biased'],default='ideal')
    parser.add_argument('--duration',type=float,default=5.0,help='stationary observation in simulated seconds')
    parser.add_argument('--timeout',type=float,default=90.0,help='wall deadline for each bounded wait')
    parser.add_argument('--steps',type=int,default=1000)
    parser.add_argument('--output',default='/workspace/evidence/latest.json')
    result=parser.parse_args()
    if result.duration<1 or result.duration>60 or not 0<result.timeout<=300 or not 1<=result.steps<=10000:
        parser.error('duration 1..60 sim s, timeout 0..300 wall s, steps 1..10000')
    if result.seed<1 or result.seed>4294967295:
        parser.error('seed must be a nonzero uint32')
    if result.action=='control' and result.operation is None:
        parser.error('control needs pause|play|step|reset')
    result.namespace,result.frame_prefix,result.world_name=names(result.namespace,result.frame_prefix,result.world_name)
    return result


@contextmanager
def fresh_launch(args, log_path):
    """Own only the process group created here; always wait for shutdown."""
    with open('/tmp/l04-session.lock','w') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('An L04 launch is already running; stop it before check.')
        with open(log_path,'w') as log:
            launch_args=[f'{k}:={v}' for k,v in {
                'mode':args.mode,'namespace':args.namespace,'frame_prefix':args.frame_prefix,
                'world_name':args.world_name,'seed':args.seed,'target_rtf':args.target_rtf,
                'profile':args.profile,'gui':'false','rviz':'false'}.items()]
            child=subprocess.Popen(['ros2','launch','--noninteractive','rover_sim_lab','sim.launch.py',*launch_args],
                                   stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                yield child
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try: os.killpg(child.pid,signal.SIGTERM)
                    except ProcessLookupError: pass
                    try: child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        try: os.killpg(child.pid,signal.SIGKILL)
                        except ProcessLookupError: pass
                        child.wait(timeout=3)
                cleanup_session(child.pid)


def experiment(args, report, child=None):
    import rclpy
    from .observer import Observer, practice_metrics, evaluate
    rclpy.init(args=[])
    observer=Observer(args)
    # A crashed bringup should fail promptly instead of consuming the wall deadline.
    pump=observer.pump
    def checked_pump(duration=0.05):
        if child is not None and child.poll() is not None:
            raise RuntimeError(f'bringup exited {child.returncode}; inspect launch_log')
        pump(duration)
    observer.pump=checked_pump
    try:
        if args.action=='control':
            observer.until(lambda:observer.stats is not None,'world stats')
            if args.operation=='step':
                observer.control('pause')
                acknowledged=observer.stats_serial
                observer.until(lambda:observer.stats_serial>acknowledged and observer.stats['paused'],
                               'fresh paused stats before stepping')
            before=observer.stats.copy()
            observer.control(args.operation,args.steps)
            acknowledged=observer.stats_serial
            if args.operation=='step':
                observer.until(lambda:observer.stats_serial>acknowledged and
                    observer.stats['iterations']>=before['iterations']+args.steps and observer.stats['paused'],'step')
                if (observer.stats['iterations']-before['iterations']!=args.steps or
                    abs(observer.stats['sim_time']-before['sim_time']-0.001*args.steps)>1e-8):
                    raise RuntimeError(f'Incorrect step result: before={before}, after={observer.stats}')
            elif args.operation=='reset':
                observer.until(lambda:observer.stats_serial>acknowledged and
                    observer.stats['iterations']==0 and abs(observer.stats['sim_time'])<1e-9 and
                    observer.stats['paused'],'fresh reset stats')
            else:
                observer.until(lambda:observer.stats_serial>acknowledged and
                    observer.stats['paused']==(args.operation=='pause'),'fresh pause/play stats')
            report.update(before=before,after=observer.stats.copy(),success=True,completed=True)
            return
        samples,window=observer.collect(args.duration)
        report.update(streams=samples,window=window,checks=evaluate(observer,samples))
        if args.mode=='homework':
            from .homework_metrics import analyze_samples
            metrics=analyze_samples
        else:
            metrics=practice_metrics
        report['metrics']={key:metrics(records) for key,records in samples.items()}
        if args.action in ('scenario','check'):
            phases=observer.scenario()
            report['phases']=phases
            report['checks'] += [row for row in evaluate(observer,samples,phases) if row['category'] in ('motion','semantics')]
        report['success']=all(row['ok'] for row in report['checks'])
        report['completed']=True
    finally:
        if args.action!='control':
            try: observer.set_velocity(barrier=False)
            except Exception: pass
        report.setdefault('streams',dict(observer.streams))
        report['final_stats']=observer.stats
        observer.destroy_node()
        rclpy.shutdown()


def main():
    args=arguments()
    output=Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True,exist_ok=True)
    report={'schema_version':1,'action':args.action,'mode':args.mode,'namespace':args.namespace,
            'frame_prefix':args.frame_prefix,'world_name':args.world_name,'parameters':vars(args),
            'completed':False,'success':False,'checks':[],
            'environment':{'architecture':platform.machine(),'ros_distro':os.environ.get('ROS_DISTRO'),
                           'domain':os.environ.get('ROS_DOMAIN_ID'),'gz_partition':os.environ.get('GZ_PARTITION')},
            'started_unix':time.time()}
    try:
        if args.action=='check':
            validate_settings(load_settings(get_package_share_directory('rover_sim_lab'),args.mode))
            log_path=output.with_suffix('.launch.log')
            report['launch_log']=str(log_path)
            with fresh_launch(args,log_path) as child:
                experiment(args,report,child)
        else:
            experiment(args,report)
    except (Exception,KeyboardInterrupt) as error:
        report.update(error=f'{type(error).__name__}: {error}',success=False)
        report['traceback']=traceback.format_exc()
    finally:
        report['finished_unix']=time.time()
        output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'success':report['success'],'completed':report['completed'],'output':str(output),
                      'failed':[r['name'] for r in report['checks'] if not r['ok']],
                      'error':report.get('error')},ensure_ascii=False,indent=2))
    return 0 if report['success'] else 1


if __name__=='__main__':
    sys.exit(main())
