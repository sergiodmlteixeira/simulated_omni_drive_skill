#!/bin/bash

aws_setup_bundle()
{
    echo "Setting up bundle..."
    mkdir ~/bundle_ws && cd ~/bundle_ws 
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
    source /opt/ros/melodic/setup.bash
    colcon build --base-paths ~/target_ws ~/upstream_ws ~/downstream_ws
    pip3 install -U setuptools pip
    pip3 install colcon-ros-bundle
    colcon bundle --base-paths ~/target_ws ~/upstream_ws ~/downstream_ws
}

aws_upload_bundle()
{
    echo "Uploading bundle to AWS S3 Bucket..."
    aws s3 cp bundle/output.tar $AWS_ROBOT_BUNDLE --no-progress
    echo "Bundle Uploaded!"
    export TERM=xterm
}

aws_simulation()
{
    echo "Starting Simulation..."
    aws lambda invoke --function-name=$AWS_SIM_FUNCTION --payload '{ "package": "simulated_omni_drive_server", "launch_file": "run_sim_test.launch" }' response.json
    sleep 10
    temp=$(aws stepfunctions list-executions  --state-machine-arn=$AWS_SF_ARN --max-items=1 --query executions[0].executionArn)
    temp="${temp%\"}"
    SF_EXECUTION="${temp#\"}"
    echo "Simulation is running on AWS Robomaker..."
    while true
    do
        temp=$(aws stepfunctions describe-execution --execution-arn=$SF_EXECUTION --query "status")
        temp="${temp%\"}"
        STATUS="${temp#\"}"
        if [[ "$STATUS" == "SUCCEEDED" ]]; then
            echo "Simulation finished successfully"
            break
        elif [[ "$STATUS" == "RUNNING" ]]; then
            sleep 10
        else
            echo "Simulation failed"
            exit 1
        fi
    done
}

aws_setup_bundle
aws_upload_bundle
aws_simulation
