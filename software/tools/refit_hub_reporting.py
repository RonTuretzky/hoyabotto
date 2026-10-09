"""Rate-limited background uploads that cannot terminate the training loop."""
from concurrent.futures import ThreadPoolExecutor
from collections import OrderedDict
from email.utils import parsedate_to_datetime
import json
import re
import time


def parse_training_progress(text, target_steps=25000, resume_step=0):
    """Convert native tqdm counts (remaining steps on resume) to absolute steps."""
    result = {}
    for match in re.finditer(r'(\d+)/(\d+) \[([^]]+)\]', text):
        done, total = int(match[1]), int(match[2])
        if total not in (target_steps, target_steps-resume_step) or done > total:
            continue
        result = dict(training_step=target_steps-total+done,
                      training_progress_text=match[0])
        rate = re.search(r'([\d.]+)(step/s|s/step)', match[3])
        if rate and float(rate[1]) > 0:
            result['training_seconds_per_step'] = (1/float(rate[1]) if rate[2] == 'step/s'
                                                   else float(rate[1]))
    return result


def retry_delay(exc, now):
    response = getattr(exc, 'response', None)
    headers = getattr(response, 'headers', {}) or {}
    value = headers.get('Retry-After') or headers.get('retry-after')
    if value:
        try:
            return max(0., float(value))
        except ValueError:
            try:
                return max(0., parsedate_to_datetime(value).timestamp()-now)
            except (TypeError, ValueError, OverflowError):
                pass
    if getattr(response, 'status_code', None) == 429:
        # HF's repository-commit limit reports an approximately one-hour cooldown.
        return 3600.
    return 300.


class DeferredUploads:
    """Single upload at a time, coalescing telemetry and ordinary checkpoints.

    A failed callback stays queued. HTTP failures affect delivery, never the
    optimizer or process supervisor. Final flush explicitly requires delivery.
    """
    def __init__(self, blocked_until=0., interval=60., clock=time.time, executor=None):
        self.clock = clock
        self.next_attempt = blocked_until
        self.interval = interval
        self.pending = OrderedDict()
        self.completed = set()
        self.executor = executor or ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.active = None
        self.last_error = None

    def enqueue(self, key, callback):
        self.completed.discard(key)
        self.pending[key] = callback

    def tick(self):
        now = self.clock()
        if self.future is not None and self.future.done():
            key, callback = self.active
            try:
                self.future.result()
                self.completed.add(key)
                self.last_error = None
                self.next_attempt = max(self.next_attempt, now+self.interval)
            except Exception as exc:
                self.last_error = f'{type(exc).__name__}: {exc}'[:1200]
                self.next_attempt = max(self.next_attempt, now+retry_delay(exc, now))
                # A newer coalesced callback wins over an older failed upload.
                if key not in self.pending:
                    self.pending[key] = callback
                print(json.dumps(dict(upload_deferred=True, error=self.last_error,
                                      retry_after_epoch=self.next_attempt)), flush=True)
            self.future = self.active = None
        if self.future is None and self.pending and now >= self.next_attempt:
            self.active = self.pending.popitem(last=False)
            self.future = self.executor.submit(self.active[1])

    def flush(self, deadline):
        while self.pending or self.future is not None:
            self.tick()
            if self.clock() > deadline:
                raise TimeoutError(f'Upload deadline reached; {self.last_error or "pending artifacts"}')
            time.sleep(1)
        self.executor.shutdown(wait=True)
