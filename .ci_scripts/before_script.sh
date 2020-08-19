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
mkdir .ssh/
echo -e "$SSH_PRIVATE_KEY" > .ssh/id_ed25519
chmod 600 .ssh/id_ed25519

touch .ssh/known_hosts
ssh-keyscan github.com >> .ssh/known_hosts
ssh-keyscan gitlab.inesctec.pt >> .ssh/known_hosts