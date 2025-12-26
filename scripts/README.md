# Scripts

These scripts exist so you don’t have to keep typing the same long commands over and over.

Instead of remembering Git and ROS commands, you run one short command and move on.

## One-time setup

Make the scripts runnable:
bash
`chmod +x scripts/*.sh`
## What each script does
**reset_repo.sh**
Instead of deleting your repo, cloning again, and fixing SSH every time, run:

`./scripts/reset_repo.sh`

This makes your local repo look exactly like what’s on GitHub main.

### ros_env.sh

Instead of typing:
```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Just run:
`source scripts/ros_env.sh`
It sets up ROS, builds the workspace if needed, and makes URDF/Xacro changes update instantly.

## Optional: aliases (type even less)

If you don’t want to type `source scripts/...` every time, you can add aliases.

Open your shell config file:
- Bash: `~/.bashrc`
- Zsh: `~/.zshrc`

Add these lines:
```bash
alias roson='source scripts/ros_env.sh'
```
Reload the file:
`source ~/.bashrc   # or ~/.zshrc`

Now you can just type:
roson


