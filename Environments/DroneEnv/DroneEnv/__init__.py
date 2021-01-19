from gym.envs.registration import register

register(
    id='drone-v0',
    entry_point='DroneEnv.envs:ArduPilotContinuousTrainer',
    kwargs={
        'other_ip': 'other_ip',
        'lim': 'lim',
        'dest': 'dest',
        'adverse': 'adverse',
        'reward_fn': 'reward_fn'
    }
)
register(
    id='drone-v1',
    entry_point='DroneEnv.envs:ArduPilotContinuousTrainer2',
    kwargs={
        'other_ip': 'other_ip',
        'lim': 'lim',
        'dest': 'dest',
        'adverse': 'adverse',
        'reward_fn': 'reward_fn'
    }
)
register(
    id='drone-v2',
    entry_point='DroneEnv.envs:ArduPilotMultipleSensors',
    kwargs={
        'other_ip': 'other_ip',
        'lim': 'lim',
        'dest': 'dest',
        'adverse': 'adverse',
        'reward_fn': 'reward_fn'
    }
)