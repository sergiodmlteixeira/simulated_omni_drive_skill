#!/bin/bash

aws_main()
{
    if [[ "$AWS" == "true" ]]; then
        aws_cli_install
        aws_configuration
    fi
}

aws_cli_install()
{
    cd ~ && mkdir .temp_aws && cd .temp_aws
    curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
    unzip awscliv2.zip
    ./aws/install
}

aws_configuration()
{
    aws configure set aws_access_key_id $AWS_ACCESS_KEY
    aws configure set aws_secret_access_key $AWS_SECRET_KEY
    aws configure set default.region $AWS_REGION
}

aws_main
