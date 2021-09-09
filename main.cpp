#include <iostream>
#include <cstdlib>
#include <ctime>
#include "win32_window.h"
#include "render_device.h"

// ��¼�����Ƿ��Ѿ������£�switchAndShowMode��ʹ��
int spaceHitRight = 0;
int spaceHitLeft = 0;

const double camMoveSpeed = 2.0;	// ����ƶ��ٶ�
const double camRotateSpeed = 3.0;	// �����ת�ٶ�
double camPitchAngle = 0.0;			// ���������
double camYawAngle = 0.0;			// ���ƫ����

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
	// ��ʼ�����ڲ����ñ���
	const int WIDTH = 800, HEIGHT = 600;
	if (screen_init(WIDTH, HEIGHT, TEXT("My soft renderer - [WASDQE]: move, \
		[mouse]: rotate, [left/right]: switch render mode")))
		return -1;

	// ��ʼ����Ⱦ�豸
	RenderDevice device(WIDTH, HEIGHT, screen_fb);

	// ����ģ�ͺ�������ͼ
	Model diablo("diablo3_pose.obj");
	device.texture.loadBmpTexture("diablo3_pose_diffuse.bmp");
	Model cube("cube.obj");

	// ���ɶ��������ĳ���
	Scene cubeScene;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			cube.worldMatrix.setTranslate(2.0 * i, 0.0, 2.0 * j);
			cubeScene.models.push_back(cube);
		}
	}

	// ����Ⱦѭ��
	while (screen_exit == 0 && screen_keys[VK_ESCAPE] == 0) {
		screen_dispatch();
		calculateAndShowFPS();						// ���㲢��ʾ֡��
		switchAndShowMode(device);					// �ո���л���Ⱦģʽ������ʾ��Ⱦģʽ
		controlCamera(device.camera);				// ��������������ת��λ��
		device.refresh(Vector3(1.0, 0.95, 0.8));	// ˢ�±任������Ȼ��桢����
		//device.drawScene(cubeScene);				// ��Ⱦ�����峡��
		//device.showMipmap(2);
		device.drawModel(diablo);
		screen_update();
		SetCursorPos(screenCenterPosX, screenCenterPosY); // ��ԭ��굽��Ļ����
	}
	return 0;
}

// ���㲢��ʾ֡��
inline void calculateAndShowFPS() {
	QueryPerformanceCounter(&nowTime);
	elapsedMicroseconds.QuadPart = (nowTime.QuadPart - lastTime.QuadPart);
	lastTime.QuadPart = nowTime.QuadPart;
	elapsedMicroseconds.QuadPart *= 1000000;
	elapsedMicroseconds.QuadPart /= frequency.QuadPart;
	// ��ʾ֡��
	HDC hDC = GetDC(screen_handle);
	if (++frameCnt == 200) {
		frameCnt = 0;
		sprintf_s(fpsText + 5, 14, "%.0lf\n", 1.0 / ((double)elapsedMicroseconds.QuadPart / 1000000.0));
	}
	TextOutA(hDC, 0, 15, fpsText, strlen(fpsText) - 1);
	ReleaseDC(screen_handle, hDC);
}

// ��������������ת��λ��
inline void controlCamera(Camera& camera) {
	double deltatime = (double)elapsedMicroseconds.QuadPart / 1000000.0;
	// �����ƾ�ͷ��ת�������ƶ����Ƹ����ǣ������ƶ�����ƫ����
	double camRotate = deltatime * camRotateSpeed;
	camPitchAngle += camRotate * (screenCenterPosY - mousePosY);
	camPitchAngle = clamp(camPitchAngle, -90.0, 90.0);
	camYawAngle += camRotate * (screenCenterPosX - mousePosX);
	while (camYawAngle > 360.0)
		camYawAngle -= 360.0;
	while (camYawAngle < 0.0)
		camYawAngle += 360.0;
	camera.setCamera(camPitchAngle, camYawAngle);
	// WSADǰ�����ң�QE����
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

// ���ҷ�����л���Ⱦģʽ
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