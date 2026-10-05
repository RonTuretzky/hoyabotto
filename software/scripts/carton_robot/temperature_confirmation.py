"""Confirm suspect temperature readings while the selected joint is released."""

def confirm_temperature(initial, limit, release, read_sample, sleep):
    if initial <= limit:
        return []
    release()
    samples = []
    for _ in range(3):
        sleep(0.1)
        sample = read_sample()
        samples.append(sample)
        if sample['torque'] != 0:
            raise RuntimeError('Cannot verify motor release during temperature confirmation')
        if sample['status'] != 0:
            raise RuntimeError('Servo reports a fault during temperature confirmation')
        if sample['temperature'] > limit:
            raise RuntimeError('High temperature confirmed by a follow-up read')
    return samples
