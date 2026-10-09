"""Same-clock telemetry chronology; hardware clock synchronization remains U."""
import math
import numpy as np


def packet_time(sample,last_consumer,last_new_range):
    stamp=float(sample['stamp_s'])
    if not math.isfinite(stamp):raise ValueError('nonfinite sensor consumer timestamp')
    if last_consumer is not None and not 0<stamp-last_consumer<=.2:
        raise ValueError('sensor time ordering/gap invalid')
    producer=float(sample.get('range_stamp_s',stamp))
    if not math.isfinite(producer) or producer>stamp:
        raise ValueError('nonfinite or future range timestamp')
    if last_new_range is not None:
        if producer<last_new_range or (sample.get('range_new',True) and producer==last_new_range):
            raise ValueError('range producer ordering invalid')
    return stamp,producer


def available_timestamp(stamp,now,label):
    stamp=float(stamp)
    if not math.isfinite(stamp) or stamp>now:
        raise ValueError(f'nonfinite or future {label} timestamp')


def history_time(history,now,label,minimum=0):
    """Validate retained/prospective history before any estimator mutation."""
    if not math.isfinite(now):raise ValueError('nonfinite velocity fit timestamp')
    values=np.asarray(history,dtype=float)
    if len(values)==0 and minimum==0:return
    if (values.ndim!=2 or values.shape[1]!=2 or len(values)<minimum
            or not np.all(np.isfinite(values))):
        raise ValueError(f'insufficient or nonfinite {label} fit history')
    times=values[:,0]
    if not np.all(np.diff(times)>0) or times[-1]>now:
        raise ValueError(f'unordered or future {label} fit history')


def fit_history_time(ranges,imu,now):
    """Malformed history cannot fall back; unavailable IMU coverage may."""
    history_time(ranges,now,'range',minimum=2)
    history_time(imu,now,'IMU')
