"""TODO C: metrics from received measurements, independent of transport.

records is a list of dictionaries: {stamp: float seconds, arrival: monotonic
seconds, value: float gyro_z or None}. A backwards stamp starts a new epoch;
only use the newest epoch. Deduplicate stamps, retaining the first arrival.
Return keys: samples, sim_hz, wall_hz, gyro_mean, gyro_std, epochs.
epochs is zero for empty input, otherwise one plus the number of backwards
stamp jumps in the raw sequence. samples counts unique stamps in the newest
epoch. With fewer than two unique samples both rates
are None. gyro_mean and gyro_std are None without finite gyro samples;
gyro_std is population standard deviation (ddof=0). Do not mutate records.
See tests/test_homework_metrics.py for examples and docs for the contract.
"""


def analyze_samples(records):
    raise NotImplementedError('TODO C: compute newest-epoch stamp/arrival rates and gyro statistics')
