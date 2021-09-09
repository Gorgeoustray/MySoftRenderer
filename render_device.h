#pragma once

#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include "my_math.h"

typedef unsigned char BYTE;

struct Vertex {
	Vector4 pos;		// 坐标
	Color color;		// 颜色
	double diffuse;		// 漫反射系数
	double u, v;		// 纹理坐标
	double rhw;			// w的倒数，用于透视矫正(reciprocal homogenous w)

	Vertex() = default;
	Vertex(const Vector4& v, const Color& c) : pos(v), color(c) {}
	Vertex(const Vector4& v, const Color& c, const double iu, const double iv)
		: pos(v), color(c), u(iu), v(iv) {}
	Vertex(const Vector4& v, const Color& c, const double id,
		const double iu, const double iv, const double irhw)
		: pos(v), color(c), diffuse(id), u(iu), v(iv), rhw(irhw) {}

	Vertex operator + (const Vertex& rhs) const {
		return Vertex(pos + rhs.pos, color + rhs.color, diffuse + rhs.diffuse,
			u + rhs.u, v + rhs.v, rhw + rhs.rhw);
	}

	Vertex operator - (const Vertex& rhs) const {
		return Vertex(pos - rhs.pos, color - rhs.color, diffuse - rhs.diffuse,
			u - rhs.u, v - rhs.v, rhw - rhs.rhw);
	}

	void operator += (const Vertex& rhs) {
		pos += rhs.pos;
		color += rhs.color;
		diffuse += rhs.diffuse;
		u += rhs.u;
		v += rhs.v;
		rhw += rhs.rhw;
	}

	void operator -= (const Vertex& rhs) {
		pos -= rhs.pos;
		color -= rhs.color;
		diffuse -= rhs.diffuse;
		u -= rhs.u;
		v -= rhs.v;
		rhw -= rhs.rhw;
	}

	void operator *= (const double rhs) {
		pos *= rhs;
		color *= rhs;
		diffuse *= rhs;
		u *= rhs;
		v *= rhs;
		rhw *= rhs;
	}

	void operator /= (const double rhs) {
		pos /= rhs;
		color /= rhs;
		diffuse /= rhs;
		u /= rhs;
		v /= rhs;
		rhw /= rhs;
	}
};

inline Vertex operator * (const double k, const Vertex& ver) {
	return Vertex(ver.pos * k, ver.color * k, ver.diffuse * k,
		ver.u * k, ver.v * k, ver.rhw * k);
}

inline Vertex operator * (const Vertex& ver, const double k) {
	return Vertex(ver.pos * k, ver.color * k, ver.diffuse * k,
		ver.u * k, ver.v * k, ver.rhw * k);
}

inline Vertex operator / (const Vertex& ver, const double rhs) {
	return Vertex(ver.pos / rhs, ver.color / rhs, ver.diffuse / rhs,
		ver.u / rhs, ver.v / rhs, ver.rhw / rhs);
}

struct Triangle {
	Vertex vertices[3];
	Vector4 normal;

	Triangle() = default;
	Triangle(const Vertex& a, const Vertex& b, const Vertex& c) {
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		updateNormal();
	}
	Triangle(const Vertex& a, const Vertex& b, const Vertex& c,
		const double u0, const double v0,
		const double u1, const double v1,
		const double u2, const double v2) {
		vertices[0] = a;
		vertices[0].u = u0;
		vertices[0].v = v0;
		vertices[1] = b;
		vertices[1].u = u1;
		vertices[1].v = v1;
		vertices[2] = c;
		vertices[2].u = u2;
		vertices[2].v = v2;
		updateNormal();
	}

	void updateNormal() {
		Vector4 pos0 = vertices[0].pos;
		Vector4 pos1 = vertices[1].pos;
		Vector4 pos2 = vertices[2].pos;
		pos0.divideW();
		pos1.divideW();
		pos2.divideW();
		normal = crossProduct(pos2 - pos1, pos1 - pos0);
		normal.normalize();
	}

	double getArea() const {
		Vector4 a = vertices[0].pos;
		Vector4 b = vertices[1].pos;
		Vector4 c = vertices[2].pos;
		double ret = 0.5 * (a.x * b.y - b.x * a.y + b.x * c.y - c.x * b.y + c.x * a.y - a.x * c.y);
		return ret > 0 ? ret : -ret;
	}

	//double calculateTrapzoidArea(const Trapzoid& trap) {
	//	Vector4 a, b, c;
	//	if (trap.topL.pos.x == trap.topR.pos.x &&
	//		trap.topL.pos.y == trap.topR.pos.y &&
	//		trap.topL.pos.z == trap.topR.pos.z) {
	//		a = trap.topL.pos;
	//		b = trap.botR.pos;
	//		c = trap.botL.pos;
	//	} else {
	//		a = trap.topL.pos;
	//		b = trap.topR.pos;
	//		c = trap.botL.pos;
	//	}
	//	double ret = 0.5 * (a.x * b.y - b.x * a.y + b.x * c.y - c.x * b.y + c.x * a.y - a.x * c.y);
	//	return ret > 0 ? ret : -ret;
	//}
};

// 梯形，三角形光栅化时，1个三角形可上下分为1~2个梯形
struct Trapzoid {
	double t, b;		// 上底和下底的高度
	Vertex topL, topR;	// 左上、右上顶点
	Vertex botL, botR;	// 左下、右下顶点
	Vector4 normal;		// 法向量
};

struct Model {
	Matrix worldMatrix;
	std::vector<Vector3> vertices;
	std::vector<double> TextureUs;
	std::vector<double> TextureVs;
	std::vector<Vector3> normals;
	std::vector<std::vector<std::vector<int>>> faces;

	Model() = default;
	Model(const char* filename) {
		std::ifstream in;
		in.open(filename, std::ifstream::in);
		if (in.fail()) return;
		std::string line;
		while (!in.eof()) {
			std::getline(in, line);
			std::istringstream iss(line.c_str());
			char trash;
			if (line.compare(0, 2, "v ") == 0) {
				iss >> trash;
				Vector3 buffer;
				iss >> buffer.x >> buffer.y >> buffer.z;
				vertices.push_back(buffer);
			} else if (line.compare(0, 3, "vt ") == 0) {
				iss >> trash >> trash;
				double bufferU, bufferV;
				iss >> bufferU >> bufferV;
				TextureUs.push_back(bufferU);
				TextureVs.push_back(bufferV);
			} else if (line.compare(0, 3, "vn ") == 0) {
				iss >> trash >> trash;
				Vector3 buffer;
				iss >> buffer.x >> buffer.y >> buffer.z;
				normals.push_back(buffer);
			} else if (line.compare(0, 2, "f ") == 0) {
				std::vector<int> bufferVertex(3, 0);
				std::vector<std::vector<int>> bufferFace;
				iss >> trash;
				while (iss >> bufferVertex[0] >> trash >> bufferVertex[1] >> trash >> bufferVertex[2]) {
					for (int i = 0; i < 3; i++)
						--bufferVertex[i];
					bufferFace.push_back(bufferVertex);
				}
				faces.push_back(bufferFace);
			}
		}
	std:cout << filename << " successfully loaded:" << std::endl
		<< vertices.size() << " vertices" << std::endl
		<< faces.size() << " faces" << std::endl;
	worldMatrix.setIdentity();
	}
};

struct Scene {
	std::vector<Model> models;

	Scene() : models(std::vector<Model>()) {}
	Scene(const std::initializer_list<Model>& il) {
		models.resize(il.size());
		std::copy(il.begin(), il.end(), models.begin());
	}
};

struct Light {
	Color color;		// 光照颜色
	Vector4 direction;	// 直射光方向
	double ka = 0.2;	// 环境光系数
	double kd = 0.7;	// 漫反射系数
	double ks = 0.3;	// 高光系数
	double p = 8;		// 高光指数

	Light() = default;
	Light(const Color& c, const Vector4& d) : color(c), direction(d) {
		direction.normalize();
	}
};

class Camera {
	friend class TransformMatrix;
	friend ostream& operator << (ostream& os, const Camera& cam);

private:
	Vector4 eyePos;		// 相机的世界坐标
	Vector4 gaze;		// 相机z轴 (注视方向)
	Vector4 viewup;		// 相机y轴 (向上方向)
	Vector4 cameraX;	// 相机x轴
	double zn = 0.1, zf = 500.0;	// 近平面的z，远平面的z
	double foy = 90.0;				// y轴上的视场角（0~180）
	double ratio = 4.0 / 3.0;		// 近平面的宽高比，默认4:3

public:
	// 相机默认位于原点，注视z轴正方向，向上方向为y轴正方向
	Camera() : eyePos(Vector4(0.0, 0.0, 0.0, 1.0)), gaze(Vector4(0.0, 0.0, 1.0, 0.0)),
		viewup(Vector4(0.0, 1.0, 0.0, 0.0)), cameraX(Vector4(1.0, 0.0, 0.0, 0.0)) {}
	Camera(const Vector4& ie, const Vector4& ig, const Vector4& iv) : eyePos(ie), gaze(ig), viewup(iv) {
		gaze.normalize();
		viewup.normalize();
		cameraX = crossProduct(viewup, gaze);
		cameraX.normalize();
	}

	Vector4 getEyePos() const {
		return eyePos;
	}

	Vector4 getGaze() const {
		return gaze;
	}

	// 根据俯仰角和偏航角重设相机
	void setCamera(const double pitchAngle, const double yawAngle) {
		Matrix pitchMatrix, yawMatrix;
		pitchMatrix.setRotate(Color(1, 0, 0), pitchAngle * PI / 180.0);
		yawMatrix.setRotate(Color(0, 1, 0), yawAngle * PI / 180.0);
		gaze = yawMatrix * (pitchMatrix * Vector4(0.0, 0.0, 1.0, 0.0));
		viewup = yawMatrix * (pitchMatrix * Vector4(0.0, 1.0, 0.0, 0.0));
		cameraX = crossProduct(viewup, gaze);
		cameraX.normalize();
	}

	// 重设相机的eyePos，gaze，viewup
	void setCamera(const Vector4& ie = Vector4(), const Vector4& ig = Vector4(0.0, 0.0, 1.0, 0.0),
		const Vector4& iv = Vector4(1.0, 0.0, 0.0, 0.0)) {
		eyePos = ie;
		gaze = ig;
		viewup = iv;
		gaze.normalize();
		viewup.normalize();
		cameraX = crossProduct(viewup, gaze);
		cameraX.normalize();
	}

	// 沿y轴上下移动
	void moveUp(const double speed) {
		eyePos.y += speed;
	}

	// gaze在XOZ平面上的投影上前后移动
	void moveForward(const double speed) {
		Vector4 forward(gaze.x, 0.0, gaze.z, 0.0);
		forward.normalize();
		eyePos += speed * forward;
	}

	// gaze在XOZ平面上的投影上左右移动
	void moveRight(const double speed) {
		Vector4 right(gaze.z, 0.0, -gaze.x, 0.0);
		right.normalize();
		eyePos += speed * right;
	}

	// 俯仰角旋转
	void pitch(const double theta) {
		Matrix rotate;
		rotate.setRotate(cameraX, theta);
		gaze = rotate * gaze;
		viewup = crossProduct(gaze, cameraX);
		gaze.normalize();
		viewup.normalize();
	}

	// 偏航角旋转
	void yaw(const double theta) {
		Matrix rotate;
		rotate.setRotate(viewup, theta);
		gaze = rotate * gaze;
		cameraX = crossProduct(viewup, gaze);
		gaze.normalize();
		cameraX.normalize();
	}
};

inline ostream& operator << (ostream& os, const Camera& cam) {
	cout << "eye position: " << cam.eyePos << endl
		<< "gaze at: " << cam.gaze << endl
		<< "view up: " << cam.viewup << endl
		<< "cameraX: " << cam.cameraX;
	return os;
}

// 裁剪码
const int CLIP_CODE_GZ = 0x0001;	// z > z_max
const int CLIP_CODE_LZ = 0x0002;	// z < z_min
const int CLIP_CODE_IZ = 0x0004;	// z_min < z < z_max
const int CLIP_CODE_GX = 0x0010;	// x > x_max
const int CLIP_CODE_LX = 0x0020;	// x < x_min
const int CLIP_CODE_IX = 0x0040;	// x_min < x < x_max
const int CLIP_CODE_GY = 0x0100;	// y > y_max
const int CLIP_CODE_LY = 0x0200;	// y < y_min
const int CLIP_CODE_IY = 0x0400;	// y_min < y < y_max
const int CLIP_CODE_NULL = 0x0000;

// 管理当前帧的view，project矩阵
class TransformMatrix {
public:
	Matrix view;
	Matrix project;
	Matrix transform;

public:
	TransformMatrix() {
		view.setIdentity();
		project.setIdentity();
		transform.setIdentity();
	}

	// 设置view
	void setView(const Camera& camera) {
		Matrix translate;
		translate.setTranslate(-camera.eyePos.x, -camera.eyePos.y, -camera.eyePos.z);
		Matrix rotate({
			camera.cameraX.x, camera.cameraX.y, camera.cameraX.z, 0,
			camera.viewup.x, camera.viewup.y, camera.viewup.z, 0,
			camera.gaze.x, camera.gaze.y, camera.gaze.z, 0,
			0, 0, 0, 1,
			});
		view = rotate * translate;
	}

	// 设置project
	void setProject(const Camera& camera) {
		double theta = camera.foy / 2;
		double n = camera.zn, f = camera.zf;
		double t = tan(theta / 180.0 * PI) * n, b = -t;
		double r = t * camera.ratio, l = -r;
		Matrix perspToOrtho({
			n, 0, 0, 0,
			0, n, 0, 0,
			0, 0, n + f, -n * f,
			0, 0, 1, 0 });
		Matrix orthoTranslate, orthoScale;
		orthoTranslate.setTranslate(-(r + l) / 2.0, -(t + b) / 2.0, -n);
		orthoScale.setScale(2.0 / (r - l), 2.0 / (t - b), 1.0 / (f - n));
		project = orthoScale * orthoTranslate * perspToOrtho;
	}

	// 更新transform
	void updateTransform(const Camera& camera) {
		setView(camera);
		setProject(camera);
		transform = project * view;
	}

	const Matrix& getTransform() const { return transform; }
};

class Texture {
public:
	int32_t _w;			// 纹理宽度
	int32_t _h;			// 纹理高度
	BYTE** _bits;		// 颜色数据
	// mipmap相关
	std::vector<BYTE**> mipmap;		// mipmap纹理链
	std::vector<int> mipmapWidths;	// 纹理链中各级宽度
	std::vector<int> mipmapHeights;	// 纹理链中各级高度
	double gamma = 1.1;				// 提高mipmap的亮度

	Texture() = default;
	Texture(int width, int height) : _w(width), _h(height) {
		_bits = new BYTE * [_h];
		BYTE* ptr = new BYTE[3 * _w * _h];
		memset(ptr, 0, sizeof(ptr));
		for (int i = 0; i < _h; ++i) {
			_bits[i] = ptr + 3 * _w;
		}
	}
	~Texture() {
		if (_bits)
			delete[] _bits;
		_bits = NULL;
	}

	struct BITMAPINFOHEADER {
		uint32_t	biSize;
		uint32_t	biWidth;
		int32_t		biHeight;
		uint16_t	biPlanes;
		uint16_t	biBitCount;
		uint32_t	biCompression;
		uint32_t	biSizeImage;
		uint32_t	biXPelsPerMeter;
		uint32_t	biYPelsPerMeter;
		uint32_t	biClrUsed;
		uint32_t	biClrImportant;
	};

	// 读取BMP图片作为纹理
	void loadBmpTexture(const char* filename) {
		FILE* fp;
		errno_t err = fopen_s(&fp, filename, "rb");
		if (err) return;
		BITMAPINFOHEADER info;
		uint8_t header[14];
		int hr = (int)fread(header, 1, 14, fp);
		// fileheader大小必须是14字节
		if (hr != 14) { fclose(fp); return; }
		// 文件标识符必须是"BM"，即0x424D才是windows位图文件
		if (header[0] != 0x42 || header[1] != 0x4d) { fclose(fp); return; }
		hr = (int)fread(&info, 1, sizeof(info), fp);
		// infoheader大小必须是40字节
		if (hr != 40) { fclose(fp); return; }
		// 只支持24/32位两种格式
		if (info.biBitCount != 24 && info.biBitCount != 32) { fclose(fp); return; }
		// 初始化_bits
		if (_bits)
			delete[] _bits;
		_bits = NULL;
		_w = info.biWidth;
		_h = info.biHeight;
		_bits = new BYTE * [_h];
		BYTE* ptr = new BYTE[3 * _w * _h];
		memset(ptr, 0, 3 * _w * _h);
		for (int i = 0; i < _h; ++i) {
			_bits[i] = ptr + 3 * _w * i;
		}
		// 得到fileheader中的偏移量，开始读取像素数据
		uint32_t offset;
		memcpy(&offset, header + 10, sizeof(uint32_t));
		fseek(fp, offset, SEEK_SET);
		uint32_t pixelsize = (info.biBitCount + 7) / 8;
		uint32_t pitch = (pixelsize * info.biWidth + 3) & (~3);
		BYTE* buffer = new BYTE[pixelsize];
		for (int y = 0; y < (int)info.biHeight; ++y) {
			int _bitsY = y;
			for (int x = 0, _bitsX = 0; x < (int)info.biWidth; ++x, _bitsX += 3) {
				fread(buffer, pixelsize, 1, fp);
				_bits[_bitsY][_bitsX + 0] = buffer[0];
				_bits[_bitsY][_bitsX + 1] = buffer[1];
				_bits[_bitsY][_bitsX + 2] = buffer[2];
			}
			fseek(fp, pitch - info.biWidth * pixelsize, SEEK_CUR);
		}
		delete[] buffer;
		fclose(fp);
	}

	// 生成黑白格纹理
	void initBlackAndWhite() {
		// 初始化_bits
		if (_bits)
			delete[] _bits;
		_bits = NULL;
		_w = 1024;
		_h = 1024;
		_bits = new BYTE * [_h];
		BYTE* ptr = new BYTE[3 * _w * _h];
		for (int i = 0; i < _h; ++i) {
			_bits[i] = ptr + 3 * _w * i;
		}
		// 用黑白格填充_bits
		for (int y = 0; y < _h; ++y) {
			for (int x = 0; x < _w; ++x) {
				BYTE color = (y / 128) % 2
					? ((x / 128) % 2 ? BYTE(255) : BYTE(0))
					: ((x / 128) % 2 ? BYTE(0) : BYTE(255));
				_bits[y][x * 3] = _bits[y][x * 3 + 1] = _bits[y][x * 3 + 2] = color;
			}
		}
		puts("Black and white texture initialized");
	}

	// 根据texture生成mipmap
	void initMipmap() {
		const int direction[] = { 0, 0, 1, 1, 0 };	// 相邻两元素是00 01 11 10，表示4个方向
		mipmap.emplace_back(_bits);
		mipmapWidths.emplace_back(_w);
		mipmapHeights.emplace_back(_h);
		int mipmapWidth = _w / 2;
		int mipmapHeight = _h / 2;
		while (mipmapWidth && mipmapHeight) {
			BYTE** nextTexture = new BYTE * [mipmapWidth];
			BYTE* ptr = new BYTE[mipmapWidth * mipmapHeight * 3];
			for (int i = 0; i < mipmapHeight; ++i)
				nextTexture[i] = ptr + i * mipmapWidth * 3;
			for (int y = 0; y < mipmapHeight; ++y) {
				for (int x = 0; x < mipmapWidth; ++x) {
					double sumR = 0.0;
					double sumG = 0.0;
					double sumB = 0.0;
					double cnt = 0.0;
					for (int i = 0; i < 4; ++i) {	// 取上一个纹理中4个纹素的平均值
						int v = 2 * y + direction[i];
						int u = 2 * x + direction[i + 1];
						if (u < mipmapWidths.back() && v < mipmapHeights.back()) {
							++cnt;
							sumB += mipmap.back()[v][u * 3];
							sumG += mipmap.back()[v][u * 3 + 1];
							sumR += mipmap.back()[v][u * 3 + 2];
						}
					}
					sumR /= cnt;
					sumG /= cnt;
					sumB /= cnt;
					sumR *= gamma;
					sumG *= gamma;
					sumB *= gamma;
					if (sumR > 255.0)	sumR = 255.0;
					if (sumG > 255.0)	sumG = 255.0;
					if (sumB > 255.0)	sumB = 255.0;
					nextTexture[y][x * 3] = (BYTE)sumB;
					nextTexture[y][x * 3 + 1] = (BYTE)sumG;
					nextTexture[y][x * 3 + 2] = (BYTE)sumR;
				}
			}
			mipmap.emplace_back(nextTexture);
			mipmapWidths.emplace_back(mipmapWidth);
			mipmapHeights.emplace_back(mipmapHeight);
			mipmapWidth /= 2;
			mipmapHeight /= 2;
		}
		printf("Mipmap initialized, size: %d\n", mipmap.size());
	}

	// 得到指定level双线性插值的纹理颜色（必须先创建mipmap）
	Color getBilinearFilteredColor(const double iv, const double iu, const int ilevel = 0) {
		const int direction[] = { 0, 0, 1, 1, 0 };	// 相邻两元素是00 01 11 10，表示4个方向
		int level = clamp(ilevel, 0, int(mipmap.size() - 1));
		double doubleY = ((double)mipmapHeights[ilevel] - 1.0) * iv;
		double doubleX = ((double)mipmapWidths[ilevel] - 1.0) * iu;
		doubleY = clamp(doubleY, 0.0, double(mipmapHeights[ilevel]));
		doubleX = clamp(doubleX, 0.0, double(mipmapWidths[ilevel]));
		int y = int(doubleY), x = int(doubleX);
		int yNext = y + 1;
		int xNext = x + 1;
		const double weightVertical = doubleY - y;
		const double weightHorizontal = doubleX - x;
		double colors[3] = { 0.0 };
		for (int i = 0; i < 3; ++i) {
			double top = 0.0, bot = 0.0;
			if (x == mipmapWidths[level] - 1 && y == mipmapHeights[level] - 1) {
				colors[i] = mipmap[level][y][x * 3 + i];
			} else if (x == mipmapWidths[level] - 1) {
				colors[i] = weightVertical * mipmap[level][y + 1][x * 3 + i]
					+ (1.0 - weightVertical) * mipmap[level][y][x * 3 + i];
			} else if (y == mipmapHeights[level] - 1) {
				colors[i] = weightHorizontal * mipmap[level][y][x * 3 + 3 + i]
					+ (1.0 - weightHorizontal) * mipmap[level][y][x * 3 + i];
			} else {
				colors[i] = (weightHorizontal * mipmap[level][y][(x + 1) * 3 + i]
					+ (1.0 - weightHorizontal) * mipmap[level][y][x * 3 + i]) * (1.0 - weightVertical)
					+ (weightHorizontal * mipmap[level][y + 1][x * 3 + 3 + i]
						+ (1.0 - weightHorizontal) * mipmap[level][y + 1][x * 3 + i]) * weightVertical;
			}
		}
		return Color(colors[2] / 255.0, colors[1] / 255.0, colors[0] / 255.0);
	}

	// 得到指定level的三线性插值纹理颜色（必须先创建mipmap）
	Color getTrilinearFilteredColor(const double v, const double u, const double mipLevel) {
		int floorLevel = int(mipLevel);
		double ceilWeight = mipLevel - double(floorLevel);
		Color floorColor = getBilinearFilteredColor(v, u, floorLevel);
		if (floorLevel == mipmap.size() - 1)
			return floorColor;
		else {
			Color ceilColor = getBilinearFilteredColor(v, u, floorLevel + 1);
			return ceilColor * ceilWeight + floorColor * (1 - ceilWeight);
		}
	}
};

class RenderDevice {
public:
	int renderMode = 0;				// 渲染模式：0-线框 1-顶点颜色填充 2-纹理贴图
	int renderModeCnt = 2;			// 渲染模式的数量
	BYTE** framebuffer;				// 帧缓存
	double** zbuffer;				// 深度缓存
	int width, height;				// 屏幕宽度和屏幕高度
	TransformMatrix tm;				// 变换矩阵
	Camera camera;					// 摄像机
	Light light;					// 平行光
	Texture texture;				// 支持bmp读取的纹理图

	RenderDevice(const int w, const int h, BYTE* screen_fb) : width(w), height(h) {
		// 初始化framebuffer，使其绑定windows窗口的帧缓存
		framebuffer = new BYTE * [h];
		for (int i = 0; i < h; ++i)
			framebuffer[i] = screen_fb + 4 * w * i;
		// 初始化zbuffer
		zbuffer = new double* [h];
		double* ptr = new double[w * h];
		for (int i = 0; i < h; ++i)
			zbuffer[i] = ptr + w * i;
		clearZbuffer();
		// 初始化摄像机
		camera.setCamera(Vector4(3.0, 3.0, -2.0, 1.0));
		// 初始化变换矩阵
		tm.updateTransform(camera);
		// 初始化平行光
		light = Light(Color(1.0, 1.0, 1.0), Vector4(-1.2, -0.9, 1.5, 0.0));
		// 生成黑白纹理
		texture.initBlackAndWhite();
		// 生成mipmap
		texture.initMipmap();
	}

	// 显示对应level的mipmap
	void showMipmap(int mipmapLevel) {
		Color c;
		for (int y = 0; y < texture.mipmapHeights[mipmapLevel]; ++y) {
			for (int x = 0; x < texture.mipmapWidths[mipmapLevel]; ++x) {
				c = Color(double(texture.mipmap[mipmapLevel][y][x * 3]) / 255.0,
					double(texture.mipmap[mipmapLevel][y][x * 3 + 1]) / 255.0,
					double(texture.mipmap[mipmapLevel][y][x * 3 + 2]) / 255.0);
				drawPixel(y, x, c);
			}
		}
	}

	// 切换渲染模式
	void switchModeUp() {
		if (++renderMode == renderModeCnt)
			renderMode = 0;
	}

	void switchModeDown() {
		if (--renderMode < 0)
			renderMode = renderModeCnt - 1;
	}

	// 刷新render device的各种状态
	void refresh(Color backgroundColor) {
		tm.updateTransform(camera);
		clearZbuffer();
		clearFramebuffer(backgroundColor);
	}

	// 绘制场景
	void drawScene(const Scene& scene) {
		for (const auto& mod : scene.models)
			drawModel(mod);
	}

	// 绘制模型
	void drawModel(const Model& model) {
		int faceSize = model.faces.size();
		vector<Triangle> willBeRendered;
		// 组装三角形，进行MVP变换，裁剪，生成要渲染的三角形序列willBeRendered
		for (int i = 0; i < faceSize; ++i) {
			Triangle tri = assembleTriangleFromFace(model, i);	// 从faces的信息中组装三角形
			triangleInitDiffuse(tri, light.direction);			// 计算顶点的漫反射系数
			triangleModelTransform(tri, model.worldMatrix);		// model transform
			//if (renderMode != 0 && needFaceCulling(tri))		// 背面剔除
			//	continue;
			triangleViewTransform(tri, model.worldMatrix);		// view transform
			triangleProjectTransform(tri, model.worldMatrix);	// project transform
			if (!triangleClipping(tri, willBeRendered))			// 裁剪
				willBeRendered.emplace_back(tri);				// 将没被剔除的三角形加进渲染序列
		}
		// 渲染三角形序列
		int renderSize = willBeRendered.size();
		for (int i = 0; i < renderSize; ++i) {
			triangleViewportTransform(willBeRendered[i]);			// 视口变换
			triangleInitRhw(willBeRendered[i]);						// 生成rhw，为透视矫正做准备
			if (renderMode == 0) {
				drawTriangleWireframe(willBeRendered[i]);
			} else {
				// 计算该三角形的mipLevel
				double mipLevel;
				if (renderMode > 2) {
					Triangle textureTri;
					for (int j = 0; j < 3; ++j) {
						double w = 1.0 / willBeRendered[i].vertices[j].rhw;
						textureTri.vertices[j].pos = Vector4(
							willBeRendered[i].vertices[j].u * w,
							willBeRendered[i].vertices[j].v * w, 0.0, 0.0);
					}
					mipLevel = calculateMipLevel(willBeRendered[i].getArea(),
						textureTri.getArea() * texture._h * texture._w);
				}
				// 拆分成上下梯形分别渲染
				vector<Trapzoid> traps = splitTriangle(willBeRendered[i]);
				for (int i = 0; i < traps.size(); ++i) {
					if (renderMode > 2)
						drawTrapzoid(traps[i], mipLevel);
					else
						drawTrapzoid(traps[i]);
				}
			}
		}
	}

	// 根据三角形面积计算MipLevel
	double calculateMipLevel(const double triArea, const double textureArea) const {
		double mipLevel = log(sqrt(textureArea / triArea)) / log(2.0);
		mipLevel = clamp(mipLevel, 0.0, double(texture.mipmap.size() - 1));
		//printf("triArea = %.1lf, mipLevel = %.1lf\n", textureArea, triArea, mipLevel);
		return mipLevel;
	}

	// 显示纹理
	void showTexture() {
		Color c;
		for (int y = 0; y < texture._h; ++y) {
			for (int x = 0; x < texture._w; ++x) {
				c = Color(double(texture._bits[y][x * 3 + 2]) / 255.0,
					double(texture._bits[y][x * 3 + 1]) / 255.0,
					double(texture._bits[y][x * 3]) / 255.0);
				drawPixel(y, x, c);
			}
		}
	}

private:
	Triangle assembleTriangleFromFace(const Model& model, const int index) {
		Triangle tri;
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].pos = Vector4(
				model.vertices[model.faces[index][i][0]].x,
				model.vertices[model.faces[index][i][0]].y,
				model.vertices[model.faces[index][i][0]].z,
				1.0);
			tri.vertices[i].u = model.TextureUs[model.faces[index][i][1]];
			tri.vertices[i].v = model.TextureVs[model.faces[index][i][1]];
		}

		return tri;
	}

	bool triangleClipping(Triangle& tri, vector<Triangle>& willBeRendered) {
		int clipCodes[3] = { 0 };
		for (int i = 0; i < 3; ++i) {
			// 收集各顶点x分量信息
			if (tri.vertices[i].pos.x <= -tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_LX;
			else if (tri.vertices[i].pos.x >= tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_GX;
			else
				clipCodes[i] |= CLIP_CODE_IX;
			// 收集各顶点y分量信息
			if (tri.vertices[i].pos.y <= -tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_LY;
			else if (tri.vertices[i].pos.y >= tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_GY;
			else
				clipCodes[i] |= CLIP_CODE_IY;
			// 收集各顶点z分量信息（只考虑近平面的裁剪，不考虑远平面）
			if (tri.vertices[i].pos.z < 0.0)
				clipCodes[i] |= CLIP_CODE_LZ;
			else
				clipCodes[i] |= CLIP_CODE_IZ;
		}

		// 裁剪掉三个顶点同时在左（右、上、下）平面外的三角形
		if ((clipCodes[0] & CLIP_CODE_GX) == CLIP_CODE_GX &&
			(clipCodes[1] & CLIP_CODE_GX) == CLIP_CODE_GX &&
			(clipCodes[2] & CLIP_CODE_GX) == CLIP_CODE_GX) {
			return true;
		}
		if ((clipCodes[0] & CLIP_CODE_LX) == CLIP_CODE_LX &&
			(clipCodes[1] & CLIP_CODE_LX) == CLIP_CODE_LX &&
			(clipCodes[2] & CLIP_CODE_LX) == CLIP_CODE_LX) {
			return true;
		}
		if ((clipCodes[0] & CLIP_CODE_GY) == CLIP_CODE_GY &&
			(clipCodes[1] & CLIP_CODE_GY) == CLIP_CODE_GY &&
			(clipCodes[2] & CLIP_CODE_GY) == CLIP_CODE_GY) {
			return true;
		}
		if ((clipCodes[0] & CLIP_CODE_LY) == CLIP_CODE_LY &&
			(clipCodes[1] & CLIP_CODE_LY) == CLIP_CODE_LY &&
			(clipCodes[2] & CLIP_CODE_LY) == CLIP_CODE_LY) {
			return true;
		}

		// 处理近平面上的裁剪
		int outCnt = 0;	// 统计有几个顶点在近平面后面
		for (int i = 0; i < 3; ++i) {
			if ((clipCodes[i] & CLIP_CODE_LZ) == CLIP_CODE_LZ) {
				++outCnt;
			}
		}
		// 3个顶点都在近平面后面，直接丢弃
		if (outCnt == 3)
			return true;
		// 3个顶点都在近平面前面，直接保留
		else if (outCnt == 0)
			return false;
		// 2个顶点在近平面后面，丢弃原三角形，为渲染序列加入1个小三角形
		else if (outCnt == 2) {
			// 获得三个顶点的数据
			Vertex inPoint, outPoint1, outPoint2;
			for (int i = 0; i < 3; ++i) {
				if ((clipCodes[i] & CLIP_CODE_IZ) == CLIP_CODE_IZ) {
					// 按照顺序获得三个顶点数据，保证绕序不变
					inPoint = tri.vertices[i];
					outPoint1 = tri.vertices[(i + 1 + 3) % 3];
					outPoint2 = tri.vertices[(i + 2 + 3) % 3];
					break;
				}
			}
			double t1 = inPoint.pos.z / (inPoint.pos.z - outPoint1.pos.z);
			double t2 = inPoint.pos.z / (inPoint.pos.z - outPoint2.pos.z);
			// 线段[inPoint, outPoint1]、[inPoint, outPoint2]和近平面的交点
			Vertex intersection1 = interp(inPoint, outPoint1, t1);
			Vertex intersection2 = interp(inPoint, outPoint2, t2);
			// 加入裁剪后的小三角形
			willBeRendered.emplace_back(inPoint, intersection1, intersection2);
			return true;
			// 1个顶点在近平面后面，丢弃原三角形，为渲染序列加入2个小三角形
		} else if (outCnt == 1) {
			Vertex inPoint1, inPoint2, outPoint;
			for (int i = 0; i < 3; ++i) {
				if ((clipCodes[i] & CLIP_CODE_LZ) == CLIP_CODE_LZ) {
					// 按照顺序获得三个顶点数据，保证绕序不变
					outPoint = tri.vertices[i];
					inPoint1 = tri.vertices[(i + 1 + 3) % 3];
					inPoint2 = tri.vertices[(i + 2 + 3) % 3];
					break;
				}
			}
			double t1 = inPoint1.pos.z / (inPoint1.pos.z - outPoint.pos.z);
			double t2 = inPoint2.pos.z / (inPoint2.pos.z - outPoint.pos.z);
			// 线段[inPoint1, outPoint]、[inPoint2, outPoint]和近平面的交点
			Vertex intersection1 = interp(inPoint1, outPoint, t1);
			Vertex intersection2 = interp(inPoint2, outPoint, t2);
			// 加入裁剪后的2个小三角形
			willBeRendered.emplace_back(inPoint1, inPoint2, intersection1);
			willBeRendered.emplace_back(intersection1, inPoint2, intersection2);
			return true;
		}
		return false;
	}

	// 判断三角形是否被裁剪掉。两种返回false的情况：
	// 1. 三角形和近平面相交
	// 2. 三角形三个顶点都在视景体外面
	bool triangleClipped(Triangle& tri) {
		int falseCnt = 0;
		for (int i = 0; i < 3; ++i)
			falseCnt += checkTransform(tri.vertices[i].pos);
		return falseCnt > 2;
	}

	int checkTransform(const Vector4& p) {
		if (p.z <= 0.0)
			return 100;
		if (p.x < -p.w || p.x > p.w)
			return 1;
		if (p.y < -p.w || p.y > p.w)
			return 1;
		if (p.z > p.w)
			return 1;
		return 0;
	}

	// 返回该三角形是否被背面剔除
	bool needFaceCulling(Triangle& tri) {
		tri.updateNormal();
		Vector4 pos = tri.vertices[0].pos;
		pos.divideW();
		return dotProduct(tri.normal, camera.getEyePos() - pos) <= 0.0;
	}

	// 根据光照信息计算顶点的漫反射系数
	void triangleInitDiffuse(Triangle& tri, const Vector4& lightDirection) {
		tri.updateNormal();
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].diffuse = light.kd * (max(0.0, dotProduct(tri.normal, -light.direction)));
			//printf("%llf\n", tri.vertices[i].diffuse);
		}
	}

	// MVP变换
	void triangleMVPTransform(Triangle& tri, const Matrix& worldMatrix) {
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].pos = tm.getTransform() * (worldMatrix * tri.vertices[i].pos);
		}
	}

	void triangleModelTransform(Triangle& tri, const Matrix& worldMatrix) {
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].pos = worldMatrix * tri.vertices[i].pos;
		}
	}

	void triangleViewTransform(Triangle& tri, const Matrix& worldMatrix) {
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].pos = tm.view * tri.vertices[i].pos;
		}
	}

	void triangleProjectTransform(Triangle& tri, const Matrix& worldMatrix) {
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].pos = tm.project * tri.vertices[i].pos;
		}
	}

	// 视口变换
	void triangleViewportTransform(Triangle& tri) {
		for (int i = 0; i < 3; ++i) {
			viewportTransform(tri.vertices[i].pos);
		}
	}

	void viewportTransform(Vector4& point) {
		double rhw = 1.0 / point.w;
		point.x = (point.x * rhw + 1.0) * width * 0.5;
		point.y = (-point.y * rhw + 1.0) * height * 0.5;
		point.z = point.z * rhw;
	}

	// 生成rhw，并准备透视矫正
	void triangleInitRhw(Triangle& tri) {
		for (int i = 0; i < 3; ++i) {
			initRhw(tri.vertices[i]);
		}
	}

	void initRhw(Vertex& ver) {
		double rhw = 1.0 / ver.pos.w;
		ver.rhw = rhw;
		ver.color *= rhw;
		ver.u *= rhw;
		ver.v *= rhw;
	}

	// 绘制三角形线框，默认黑色
	void drawTriangleWireframe(const Triangle& triangle, const Color& color = Color::black()) {
		const Vertex& a = triangle.vertices[0], & b = triangle.vertices[1], & c = triangle.vertices[2];
		// 线框模式，不考虑深度
		drawLine(a.pos.y + 0.5, a.pos.x + 0.5, b.pos.y + 0.5, b.pos.x + 0.5, color);
		drawLine(b.pos.y + 0.5, b.pos.x + 0.5, c.pos.y + 0.5, c.pos.x + 0.5, color);
		drawLine(c.pos.y + 0.5, c.pos.x + 0.5, a.pos.y + 0.5, a.pos.x + 0.5, color);
	}

	// 在帧缓存绘制直线（单一颜色）
	void drawLine(const int y1, const int x1, const int y2, const int x2, const Color& c) {
		if (y1 == y2 && x1 == x2) {
			drawPixel(y1, x1, c);
		} else if (y1 == y2) {
			int inc = x2 > x1 ? 1 : -1;
			for (int x = x1; x != x2; x += inc)
				drawPixel(y1, x, c);
		} else if (x1 == x2) {
			int inc = y2 > y1 ? 1 : -1;
			for (int y = y1; y != y2; y += inc)
				drawPixel(y, x1, c);
		} else {
			// 基于Bresenham算法
			int X1 = x1, X2 = x2, Y1 = y1, Y2 = y2;
			bool steep = abs(Y2 - Y1) > abs(X2 - X1);
			if (steep) {
				std::swap(X1, Y1);
				std::swap(X2, Y2);
			}
			if (X1 > X2) {
				std::swap(X1, X2);
				std::swap(Y1, Y2);
			}
			int y = Y1, x = X1, dy = Y2 - Y1, dx = X2 - X1;
			int incY = dy > 0 ? 1 : -1;
			dy = abs(dy);
			int p = 2 * dy - dx;
			for (; x < X2; ++x) {
				if (steep)
					drawPixel(x, y, c);
				else
					drawPixel(y, x, c);
				if (p > 0) {
					y += incY;
					p -= 2 * dx;
				}
				p += 2 * dy;
			}
		}
	}

	// 把三角形分成1~2个梯形
	vector<Trapzoid> splitTriangle(Triangle& tri) {
		Vertex& a = tri.vertices[0], & b = tri.vertices[1], & c = tri.vertices[2];
		// 排序使得三个点从上到下是a,b,c
		if (a.pos.y > b.pos.y) std::swap(a, b);
		if (a.pos.y > c.pos.y) std::swap(a, c);
		if (b.pos.y > c.pos.y) std::swap(b, c);
		// 特判三点一线的情况
		if (a.pos.y == b.pos.y && a.pos.y == c.pos.y) return vector<Trapzoid>();
		if (a.pos.x == b.pos.x && a.pos.x == c.pos.x) return vector<Trapzoid>();
		// 1个三角形的情况
		if (a.pos.y == b.pos.y) {
			if (a.pos.x > b.pos.x) std::swap(a, b);
			return vector<Trapzoid>{ Trapzoid{ a.pos.y, c.pos.y, a, b, c, c, tri.normal } };
		}
		if (b.pos.y == c.pos.y) {
			if (b.pos.x > c.pos.x) std::swap(b, c);
			return vector<Trapzoid>{ Trapzoid{ a.pos.y, b.pos.y, a, a, b, c, tri.normal } };
		}
		// 2个三角形的情况
		Trapzoid trap1, trap2;
		trap1.normal = trap2.normal = tri.normal;
		trap1.t = a.pos.y;
		trap1.b = b.pos.y;
		trap1.topL = trap1.topR = a;
		trap2.t = b.pos.y;
		trap2.b = c.pos.y;
		trap2.botL = trap2.botR = c;
		double k = (b.pos.y - a.pos.y) / (c.pos.y - a.pos.y);
		Vertex d = interp(a, c, k);	// 线段bd为分割线
		if (b.pos.x > d.pos.x)
			std::swap(b, d);
		trap1.botL = trap2.topL = b;
		trap1.botR = trap2.topR = d;
		return vector<Trapzoid>{ trap1, trap2 };
	}

	// 扫描线绘制梯形
	void drawTrapzoid(const Trapzoid& trap, const double mipLevel = 0.0) {
		const int t = int(trap.t + 0.5), b = int(trap.b + 0.5);
		const double dy = trap.b - trap.t;
		const Vertex verDl = (trap.botL - trap.topL) / dy;
		const Vertex verDr = (trap.botR - trap.topR) / dy;
		Vertex verL = trap.topL + (double(t) + 0.5 - trap.t) * verDl;
		Vertex verR = trap.topR + (double(t) + 0.5 - trap.t) * verDr;
		for (int y = t; y < b; ++y) {
			if (y < 0) {
				verL += verDl * (-y);
				verR += verDr * (-y);
				y = -1;
				continue;
			}
			if (y >= height)
				break;
			if (renderMode > 2)
				drawScanline(y, verL, verR, mipLevel);
			else
				drawScanline(y, verL, verR);
			verL += verDl;
			verR += verDr;
		}
	}

	// 根据纵坐标y和左右端点绘制扫描线
	void drawScanline(const int y, const Vertex& verL, const Vertex& verR, const double mipLevel = 0.0) {
		Vertex vertex = verL;
		const int xl = int(verL.pos.x + 0.5);
		const int xr = int(verR.pos.x + 0.5);
		double dx = verR.pos.x - verL.pos.x;
		if (dx == 0)
			return;
		const Vertex dv = (verR - verL) / dx;
		for (int x = xl; x < xr; ++x) {
			if (x < 0) {
				vertex += dv * (-x);
				x = -1;
				continue;
			}
			if (x >= width)
				break;
			double w = 1.0 / vertex.rhw;
			if (renderMode == 1) {
				Color color;
				int u = int(double(texture._w - 1) * vertex.u * w + 0.5);
				int v = int(double(texture._h - 1) * vertex.v * w + 0.5);
				u = clamp(u, 0, texture._w - 1);
				v = clamp(v, 0, texture._h - 1);
				u *= 3;
				color.x = texture._bits[v][u + 2] / 255.0;
				color.y = texture._bits[v][u + 1] / 255.0;
				color.z = texture._bits[v][u] / 255.0;
				drawPixel(y, x, vertex.pos.z, color * (light.ka + vertex.diffuse));
			} else if (renderMode == 2) {
				Color color = texture.getBilinearFilteredColor(vertex.v * w, vertex.u * w);
				drawPixel(y, x, vertex.pos.z, color * (light.ka + vertex.diffuse));
			} else if (renderMode == 3) {
				Color color = texture.getTrilinearFilteredColor(vertex.v * w, vertex.u * w, mipLevel);
				drawPixel(y, x, vertex.pos.z, color * (light.ka + vertex.diffuse));
			} else if (renderMode == 4) {
				double ratio = mipLevel / ((double)texture.mipmap.size() - 1.0);
				Color color(ratio, 1.0 - ratio, 0.0);
				drawPixel(y, x, vertex.pos.z, color);
			}
			vertex += dv;
		}
	}

	// 在帧缓存绘制像素
	void drawPixel(const int y, const int x, const Color& c) {
		if (y >= 0 && x >= 0 && y < height && x < width) {
			BYTE* ptr = framebuffer[y] + 4 * x;
			BYTE R = BYTE(c.x * 255.0);
			BYTE G = BYTE(c.y * 255.0);
			BYTE B = BYTE(c.z * 255.0);
			*(ptr + 0) = B;
			*(ptr + 1) = G;
			*(ptr + 2) = R;
		}
	}

	// 在帧缓存绘制像素，zbuffer算法
	void drawPixel(const int y, const int x, const double depth, const Color& c) {
		if (y >= 0 && x >= 0 && y < height && x < width) {
			if (depth < zbuffer[y][x]) {
				zbuffer[y][x] = depth;
				BYTE* ptr = framebuffer[y] + 4 * x;
				BYTE R = BYTE(c.x * 255.0);
				BYTE G = BYTE(c.y * 255.0);
				BYTE B = BYTE(c.z * 255.0);
				*(ptr + 0) = B;
				*(ptr + 1) = G;
				*(ptr + 2) = R;
			}
		}
	}

	// 用color清空帧缓存
	void clearFramebuffer(const Color& color) {
		for (int y = 0; y < height; ++y)
			for (int x = 0; x < width; ++x)
				drawPixel(y, x, color);
	}

	// 清空深度缓存
	void clearZbuffer() {
		for (int y = 0; y < height; ++y)
			for (int x = 0; x < width; ++x)
				zbuffer[y][x] = DBL_MAX;
	}
};