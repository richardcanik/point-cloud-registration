stop:
	docker-compose stop

kill:
	docker-compose kill

down:
	docker-compose down

build:
	DOCKER_BUILDKIT=1 docker-compose build

run:
	xhost +local:
	docker-compose up -d

restart:
	docker-compose stop
	docker-compose up -d

all: down build run
