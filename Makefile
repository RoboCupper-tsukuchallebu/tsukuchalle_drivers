DOCKER ?= docker
IMAGE := ddt-can-bridge-firmware:latest
# Only `image` needs the network. The build and test containers run offline.
DOCKER_RUN := $(DOCKER) run --rm --network none -v "$(CURDIR):/work" -w /work --user "$$(id -u):$$(id -g)" $(IMAGE)

.NOTPARALLEL:
.PHONY: all image test firmware clean

all: test firmware

image:
	$(DOCKER) build -t $(IMAGE) .

test: image
	$(DOCKER_RUN) sh -c 'cmake -S tests -B build/docker/tests -G Ninja \
		&& cmake --build build/docker/tests \
		&& ctest --test-dir build/docker/tests --output-on-failure \
		&& cd host && python3 -m unittest'

firmware: image
	$(DOCKER_RUN) sh -c 'cmake -S firmware -B build/docker/firmware -G Ninja \
		&& cmake --build build/docker/firmware'

clean:
	rm -rf build/docker
