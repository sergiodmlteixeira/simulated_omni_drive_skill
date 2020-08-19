# start SSH agent
# shellcheck disable=SC2046
eval $(ssh-agent -s)
# add key to agent
ssh-add <(echo "$SSH_PRIVATE_KEY") || { res=$?; echo "could not add ssh key"; exit $res; }
if [ -n "$SSH_SERVER_HOSTKEYS" ]; then
  mkdir -p ~/.ssh
  # setup known hosts
  echo "$SSH_SERVER_HOSTKEYS" > ~/.ssh/known_hosts
fi