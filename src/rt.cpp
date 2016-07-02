#include <cstdio>
#include <iostream>

#include "timer.h"

#include "camera.h"
#include "renderer.h"
#include "scene.h"
#include "scheduler.h"

int main(int /*argc*/, const char* /*argv*/[])
{
	Renderer renderer;

    workQueueInit();

    auto width = 1024;
    auto height = 768;

#if 0
	auto scene = Scene::makeCornellBox();
	auto camera = Camera(
		Vector3f(50.0f, 48.0f, 220.0f),
		normal(Vector3f(0.0f, -0.042612f, -1.0f)),
		width,
		height,
		0.785398f
	);
	scene.preprocess();
#else
	auto scene = Scene::loadFromObj("D:\\users\\marcin\\projects\\yart\\scenes\\cornell-box\\", "CornellBox-Glossy.obj");
    scene.preprocess();
	auto camera = Camera(
		Vector3f(0.0f, 0.85f, 3.0f),
		normal(Vector3f(0.0f, 0.0f, -1.0f)),
		width,
		height,
		0.785398f
	);
#endif

	Timer timer;
	timer.start();
	renderer.render(scene, camera);
	auto elapsed = timer.elapsed();

	auto nanosec = elapsed.count();
	auto minutes = nanosec / 60000000000;

	nanosec = nanosec - minutes * 60000000000;
	auto seconds = nanosec / 1000000000;

	nanosec = nanosec - seconds * 1000000000;
	auto milisec = nanosec / 1000000;

	printf("Time spent rendering: %lldm %llds %lldms\n", minutes, seconds, milisec);

	camera.saveImage("image.bmp");

	workQueueShutdown();

	return 0;
}

