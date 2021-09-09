#include <iostream>
#include <cstdlib>
#include <ctime>
#include "win32_window.h"
#include "render_device.h"

// 记录按键是否已经被按下，switchAndShowMode中使用
int spaceHitRight = 0;
int spaceHitLeft = 0;

const double camMoveSpeed = 2.0;	// 相机移动速度
const double camRotateSpeed = 3.0;	// 相机旋转速度
double camPitchAngle = 0.0;			// 相机俯仰角
double camYawAngle = 0.0;			// 相机偏航角

void calculateAndShowFPS();
void controlCamera(Camera& camera);
void switchAndShowMode(RenderDevice& device);

const std::vector<std::string> renderModeStrings({
	"wireframe",
	"no texture filtering",
	"bilinear filtering",
	"trilinear filtering",
	"show mipmap level",
	});

int main() {
	// 初始化窗口并设置标题
	const int WIDTH = 800, HEIGHT = 600;
	if (screen_init(WIDTH, HEIGHT, TEXT("My soft renderer - [WASDQE]: move, \
		[mouse]: rotate, [left/right]: switch render mode")))
		return -1;

	// 初始化渲染设备
	RenderDevice device(WIDTH, HEIGHT, screen_fb);

	// 读入模型和纹理贴图
	Model diablo("diablo3_pose.obj");
	device.texture.loadBmpTexture("diablo3_pose_diffuse.bmp");
	Model cube("cube.obj");

	// 生成多个立方体的场景
	Scene cubeScene;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			cube.worldMatrix.setTranslate(2.0 * i, 0.0, 2.0 * j);
			cubeScene.models.push_back(cube);
		}
	}

	// 主渲染循环
	while (screen_exit == 0 && screen_keys[VK_ESCAPE] == 0) {
		screen_dispatch();
		calculateAndShowFPS();						// 计算并显示帧率
		switchAndShowMode(device);					// 空格可切换渲染模式，并显示渲染模式
		controlCamera(device.camera);				// 键鼠控制摄像机旋转、位移
		device.refresh(Vector3(1.0, 0.95, 0.8));	// 刷新变换矩阵、深度缓存、背景
		//device.drawScene(cubeScene);				// 渲染立方体场景
		//device.showMipmap(2);
		device.drawModel(diablo);
		screen_update();
		SetCursorPos(screenCenterPosX, screenCenterPosY); // 还原鼠标到屏幕中心
	}
	return 0;
}

// 计算并显示帧率
inline void calculateAndShowFPS() {
	QueryPerformanceCounter(&nowTime);
	elapsedMicroseconds.QuadPart = (nowTime.QuadPart - lastTime.QuadPart);
	lastTime.QuadPart = nowTime.QuadPart;
	elapsedMicroseconds.QuadPart *= 1000000;
	elapsedMicroseconds.QuadPart /= frequency.QuadPart;
	// 显示帧率
	HDC hDC = GetDC(screen_handle);
	if (++frameCnt == 200) {
		frameCnt = 0;
		sprintf_s(fpsText + 5, 14, "%.0lf\n", 1.0 / ((double)elapsedMicroseconds.QuadPart / 1000000.0));
	}
	TextOutA(hDC, 0, 15, fpsText, strlen(fpsText) - 1);
	ReleaseDC(screen_handle, hDC);
}

// 键鼠控制摄像机旋转、位移
inline void controlCamera(Camera& camera) {
	double deltatime = (double)elapsedMicroseconds.QuadPart / 1000000.0;
	// 鼠标控制镜头旋转，上下移动控制俯仰角，左右移动控制偏航角
	double camRotate = deltatime * camRotateSpeed;
	camPitchAngle += camRotate * (screenCenterPosY - mousePosY);
	camPitchAngle = clamp(camPitchAngle, -90.0, 90.0);
	camYawAngle += camRotate * (screenCenterPosX - mousePosX);
	while (camYawAngle > 360.0)
		camYawAngle -= 360.0;
	while (camYawAngle < 0.0)
		camYawAngle += 360.0;
	camera.setCamera(camPitchAngle, camYawAngle);
	// WSAD前后左右，QE上下
	double camMove = deltatime * camMoveSpeed;
	if (screen_keys['W'])
		camera.moveForward(camMove);
	else if (screen_keys['S'])
		camera.moveForward(-camMove);
	if (screen_keys['A'])
		camera.moveRight(-camMove);
	else if (screen_keys['D'])
		camera.moveRight(camMove);
	if (screen_keys['Q'])
		camera.moveUp(camMove);
	else if (screen_keys['E'])
		camera.moveUp(-camMove);
}

// 左右方向键切换渲染模式
inline void switchAndShowMode(RenderDevice& device) {
	if (screen_keys[VK_RIGHT]) {
		if (spaceHitRight == 0) {
			spaceHitRight = 1;
			device.switchModeUp();
		}
	} else {
		spaceHitRight = 0;
	}
	if (screen_keys[VK_LEFT]) {
		if (spaceHitLeft == 0) {
			spaceHitLeft = 1;
			device.switchModeDown();
		}
	} else {
		spaceHitLeft = 0;
	}
	HDC hDC = GetDC(screen_handle);
	std::string showText = std::string("render mode: ") + renderModeStrings[device.renderMode];
	TextOutA(hDC, 0, 0, showText.c_str(), strlen(showText.c_str()));
	ReleaseDC(screen_handle, hDC);
}