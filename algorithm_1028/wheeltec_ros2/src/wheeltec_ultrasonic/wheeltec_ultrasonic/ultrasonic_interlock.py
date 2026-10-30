"""Front A-D short-range veto, using existing calibration without correction.

Channels match the installed wheeltec_ultrasonic front solver. With filtering
enabled, every near reading, including critical <=25 cm, requires consecutive-
three confirmation; a single large jump cannot erase a confirmed obstacle.
Three far samples release a confirmed obstacle. EMA is display-only;
missing, invalid or slow data still fails closed.
"""
from collections import deque
import math
import statistics


class FrontUltrasonicInterlock:
    def __init__(self,stop_distance_m=.45,despike_frames=0,channels='ABCD',direction='front'):
        if not math.isfinite(stop_distance_m) or not .1<=stop_distance_m<=1.:
            raise ValueError('Invalid test stopping distance')
        self.stop_distance_m=stop_distance_m
        if type(despike_frames) is not int or despike_frames not in (0,2):
            raise ValueError('Supported despike depth is zero or two frames')
        if not channels or any(c not in 'ABCDEF' for c in channels) or len(set(channels))!=len(channels):
            raise ValueError('Invalid ultrasonic channels')
        if direction not in ('front','rear'):raise ValueError('Invalid ultrasonic direction')
        self.direction=direction
        self.despike_frames=despike_frames
        self.critical_distance_m=min(.25,stop_distance_m)
        self.samples={k:deque(maxlen=12) for k in channels}
        self.invalid={k:False for k in channels}
        self.confirmed={k:False for k in channels}
        self.near_since={k:None for k in channels}
        self.near_run={k:0 for k in channels}
        self.far_run={k:0 for k in channels}
        self.blocked={k:None for k in channels}
        self.accepted_range={k:None for k in channels}
        self.smoothed={k:None for k in channels}

    def reset_confirmation(self,channel):
        self.confirmed[channel]=False;self.near_since[channel]=None
        self.near_run[channel]=self.far_run[channel]=0
        self.blocked[channel]=self.accepted_range[channel]=self.smoothed[channel]=None

    def update(self,channel,distance,stamp,now,minimum=.02,maximum=5.3):
        if channel not in self.samples:raise ValueError('Unknown ultrasonic channel')
        valid=(all(math.isfinite(v) for v in [distance,stamp,now,minimum,maximum])
               and 0<=now-stamp<.25 and 0<=minimum<maximum and minimum<=distance<=maximum)
        self.invalid[channel]=not valid
        if not valid:
            self.samples[channel].clear()
            self.reset_confirmation(channel)
            return
        samples=self.samples[channel]
        if samples and stamp<=samples[-1][0]:return
        if samples and stamp-samples[-1][0]>=.3:
            samples.clear();self.reset_confirmation(channel)
        samples.append((stamp,distance))
        if distance<=self.stop_distance_m:
            # Confirmation measures time available to this filter after first
            # reception. Header transport age is checked separately; using it
            # here can expire two delayed packets before a fresh third arrives.
            if self.near_since[channel] is None:self.near_since[channel]=now
            self.near_run[channel]+=1;self.far_run[channel]=0
        else:
            self.near_since[channel]=None
            self.near_run[channel]=0;self.far_run[channel]+=1
        if self.despike_frames:
            if self.near_run[channel]>=3 or self.far_run[channel]>=3:
                self.blocked[channel]=self.near_run[channel]>=3
                self.accepted_range[channel]=statistics.median(r for t,r in list(samples)[-3:])
            accepted=self.accepted_range[channel]
            if accepted is not None:
                old=self.smoothed[channel]
                self.smoothed[channel]=accepted if old is None else old+.35*(accepted-old)

    def status(self,now):
        minima={};filtered={};raw_latest={}
        def result(clear,reason,channel=None):
            out=dict(clear=clear,reason=reason,minima_m=minima,
                filtered_m=filtered,raw_latest_m=raw_latest,
                despike_frames=self.despike_frames,filter_window_frames=3 if self.despike_frames else 0,
                critical_stop_m=self.critical_distance_m if self.despike_frames else None)
            if self.despike_frames:
                out.update(persistent_near_deadline_s=.12,filter_method='consecutive_three',
                    near_confirm_frames=3,far_release_frames=3,
                    smoothed_display_m=dict(self.smoothed),display_ema_alpha=.35,
                    data_quality_window_frames=5,
                    near_run_frames=dict(self.near_run),display_smoothing_used_for_safety=False)
            if channel is not None:out['channel']=channel
            return out
        for channel,samples in self.samples.items():
            if self.invalid[channel] or not samples or not 0<=now-samples[-1][0]<.3:
                self.confirmed[channel]=False
                return result(False,'ultrasonic_missing_stale_or_invalid',channel)
            recent=[r for t,r in samples if 0<=now-t<=.25]
            if not recent:return result(False,'ultrasonic_stale',channel)
            minima[channel]=min(recent)
            raw_latest[channel]=samples[-1][1]
            if self.despike_frames:
                # All near values, including <=25 cm, require three frames.
                # The persistence deadline may extend confirmation, never bypass it.
                if (self.near_since[channel] is not None and self.near_run[channel]>=3
                        and now-self.near_since[channel]>=.12):
                    self.blocked[channel]=True
                    self.accepted_range[channel]=min(r for t,r in list(samples)[-self.near_run[channel]:])
                window=list(samples)[-5:]
                if (len(window)<5 or now-window[-1][0]>=.15 or now-window[0][0]>.70
                        or any(b[0]-a[0]>.15 for a,b in zip(window,window[1:]))):
                    self.confirmed[channel]=False
                    return result(False,'ultrasonic_filter_window_not_ready',channel)
                filtered[channel]=self.accepted_range[channel]
                if self.blocked[channel] is None:
                    return result(False,'ultrasonic_waiting_for_stable_samples',channel)
                if self.blocked[channel]:
                    return result(False,'ultrasonic_'+self.direction+'_obstacle',channel)
            elif minima[channel]<=self.stop_distance_m:
                return result(False,'ultrasonic_'+self.direction+'_obstacle',channel)
            if len(recent)>=3:self.confirmed[channel]=True
            if not self.confirmed[channel]:
                return result(False,'ultrasonic_waiting_for_stable_samples',channel)
        return result(True,'ultrasonic_'+self.direction+'_clear')
