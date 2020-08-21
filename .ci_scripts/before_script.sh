apt-get -qq install -y --no-upgrade --no-install-recommends ssh ssh-client

eval $(ssh-agent -s)
# add key to agent
ssh-add <(echo "$SSH_PRIVATE_KEY" | base64 -d) || { res=$?; echo "could not add ssh key"; exit $res; 
mkdir -p ~/.ssh
# setup known hosts
chmod 700 ~/.ssh
(echo "$SSH_SERVER_HOSTKEYS" | base64 -d)  > ~/.ssh/known_hosts
chmod 644 ~/.ssh/known_hosts