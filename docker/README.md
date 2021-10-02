## Docker Env

```sh
# build dev env
./docker/build.sh
# run dev env
./docker/run.sh
# configure
rm -rf build
./docker/run.sh cmake ..
# run make
./docker/run.sh make -j$(nproc)
# run test
./docker/run.sh make test_run
```
