#!/bin/bash

aws_main()
{

    apt-get -qq install -y --no-upgrade --no-install-recommends ssh ssh-client git

    eval $(ssh-agent -s)
    # add key to agent
    ssh-add <(echo "$SSH_PRIVATE_KEY" | base64 -d) || { res=$?; echo "could not add ssh key"; exit $res; }

    mkdir -p ~/.ssh
    # setup known hosts
    chmod 700 ~/.ssh
    (echo "$SSH_SERVER_HOSTKEYS" | base64 -d)  > ~/.ssh/known_hosts
    chmod 644 ~/.ssh/known_hosts

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
