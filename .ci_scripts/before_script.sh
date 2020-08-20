## start SSH agent
## shellcheck disable=SC2046
#eval $(ssh-agent -s)
## add key to agent
#ssh-add <(echo "$SSH_PRIVATE_KEY") || { res=$?; echo "could not add ssh key"; exit $res; }
#if [ -n "$SSH_SERVER_HOSTKEYS" ]; then
#  mkdir -p ~/.ssh
#  # setup known hosts
#  echo "$SSH_SERVER_HOSTKEYS" > ~/.ssh/known_hosts
#fi

eval $(ssh-agent -s)
ssh-add <(echo -e "$SSH_PRIVATE_KEY") || { res=$?; echo "could not add ssh key"; exit $res; }

mkdir -p ~/.ssh
chmod 700 ~/.ssh
ssh-keyscan gitlab.inesctec.pt >> ~/.ssh/known_hosts
chmod 644 ~/.ssh/known_hosts