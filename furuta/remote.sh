#!/bin/bash

# upload environment, train policy, download policy, test it.

# AWS instance IP address 
ip=""

train="deepq_p_train.py"
test="deepq_p_run.py test"

#train="ppo_p_train.py"
#test="ppo_p_run.py test"

#train="trpo_p_train.py"
#test="trpo_p_run.py test"

localpath="/Users/rene/work/git/furuta_pendulum/"
awspath="/home/ubuntu/work/git/furuta_pendulum/"

echo `date`
echo Upload project...
echo `scp -i ~/.ssh/first_aws.pem ${localpath}*.py ubuntu@${ip}:${awspath}`

echo `date`
echo Upload networks...
echo `scp -i ~/.ssh/first_aws.pem ${localpath}/tf/*.h5 ubuntu@${ip}:${awspath}/tf`

echo `date`
echo  Train policy...
#echo `ssh -i ~/.ssh/first_aws.pem ubuntu@${ip} "source /home/ubuntu/miniconda3/etc/profile.d/conda.sh && conda activate furuta-v3 && cd ${awspath} && mpirun -n 16 python ${train}"`
echo `ssh -i ~/.ssh/first_aws.pem ubuntu@${ip} "source /home/ubuntu/miniconda3/etc/profile.d/conda.sh && conda activate furuta-v3 && cd ${awspath} && python ${train}"`

echo `date`
echo Download .zips...
echo `scp -i ~/.ssh/first_aws.pem ubuntu@${ip}:${awspath}policy/*.zip /Users/rene/work/git/furuta_pendulum/policy/`
#echo Download .pkl...
#echo `scp -i ~/.ssh/first_aws.pem ubuntu@${ip}:${awspath}policy/*.pkl /Users/rene/work/git/furuta_pendulum/policy/`
echo Done
echo `date`

echo Testing...
echo `python ${test}`