## Build Create3_sim using Dockerfile

Build Docker image and run a container
```bash
cd create3_sim/docker
bash build.sh
bash run.sh
```

Inside the Docker container build the `create3_sim` project.

```bash
mkdir -p ws/src
cd ws/src
ln -s ~/create3_sim .
vcs import ~/ws/src/ < ~/ws/src/create3_sim/msgs.repos
cd ..
apt-get update
rosdep install --from-path src -yi
colcon build
```
