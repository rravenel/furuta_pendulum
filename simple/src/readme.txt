Progress notes.



pend_ppo2_tmp_1u_05r_5Tud
	13
	.1u, .5 reversal, 5/T_MAX u delta

pend_ppo2_13
	ripple as 10, 50%
	torque cost factor = 0.01

pend_ppo2_12
	11, but no ripple
	--> works on physical pendulum - poorly, inconsistently

pend_ppo2_11
	10, but max torque at .055, reverted to pos**2 cost
	--> works on physical pendulum reasonably well

pend_ppo2_10
	New physical pendulum
	Max torque .025 (3A); max ripple .012 (~1.5A)
	No reversal cost


pend_ppo2_9
	As 8
	Added torque reversal to cost; coefficient = 5

pend_ppo2_8
	As 7
	l = .300
	m = .132


pend_ppo2_7
	Same as 6, added pos cost coefficient of 3, which brings the max back to about what it was prior to 6

pend_ppo2_6
	Goes back to 4, but removed the square from the position cost component
	Hoping linear cost on position will increase 'resolution' near vertical


pend_ppo2_5
	Same, but double update step count to spend more time vertical
	--> Can't perform swing up.  Maybe need to double time steps to get same update count


pend_ppo2_4
	Again, max torque disturbance  only 1, 100%

pend_ppo2_3
	Same as pend_ppo2_2, but max torque 1.5, 100% 

pend_ppo2_2
	Below, plus random torque disturbances to simulate ripple.
	Max torque disturbance: 1
	Odds of disturbance 0.5
	500k steps

	python -m baselines.run --alg=ppo2 --env=Pendulum-v0 --num_env=4 --nminibatches=1 --noptepochs=256 --num_timesteps=500e3 --save_path=work/models/pend_ppo2_2

pend_ppo2_1
	New naming format.  Short environment indicator, algo, rev.
	Trained directly on real dims, 500K steps.
	16min

pendulum_ppo2_real_dims
	trained with steady migration from default dims

tmp
	as the name implies


Replay: python -m baselines.run --alg=ppo2 --env=Pendulum-v0 --num_timesteps=0 --load_path=work/models/pend_ppo2_2 --play