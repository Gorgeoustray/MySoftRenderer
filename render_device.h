#pragma once

#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include "my_math.h"

typedef unsigned char BYTE;

struct Vertex {
	Vector4 pos;		// ����
	Color color;		// ��ɫ
	double diffuse;		// ������ϵ��
	double u, v;		// ��������
	double rhw;			// w�ĵ���������͸�ӽ���(reciprocal homogenous w)

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

// ���Σ������ι�դ��ʱ��1�������ο����·�Ϊ1~2������
struct Trapzoid {
	double t, b;		// �ϵ׺��µ׵ĸ߶�
	Vertex topL, topR;	// ���ϡ����϶���
	Vertex botL, botR;	// ���¡����¶���
	Vector4 normal;		// ������
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
	Color color;		// ������ɫ
	Vector4 direction;	// ֱ��ⷽ��
	double ka = 0.2;	// ������ϵ��
	double kd = 0.7;	// ������ϵ��
	double ks = 0.3;	// �߹�ϵ��
	double p = 8;		// �߹�ָ��

	Light() = default;
	Light(const Color& c, const Vector4& d) : color(c), direction(d) {
		direction.normalize();
	}
};

class Camera {
	friend class TransformMatrix;
	friend ostream& operator << (ostream& os, const Camera& cam);

private:
	Vector4 eyePos;		// �������������
	Vector4 gaze;		// ���z�� (ע�ӷ���)
	Vector4 viewup;		// ���y�� (���Ϸ���)
	Vector4 cameraX;	// ���x��
	double zn = 0.1, zf = 500.0;	// ��ƽ���z��Զƽ���z
	double foy = 90.0;				// y���ϵ��ӳ��ǣ�0~180��
	double ratio = 4.0 / 3.0;		// ��ƽ��Ŀ�߱ȣ�Ĭ��4:3

public:
	// ���Ĭ��λ��ԭ�㣬ע��z�����������Ϸ���Ϊy��������
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

	// ���ݸ����Ǻ�ƫ�����������
	void setCamera(const double pitchAngle, const double yawAngle) {
		Matrix pitchMatrix, yawMatrix;
		pitchMatrix.setRotate(Color(1, 0, 0), pitchAngle * PI / 180.0);
		yawMatrix.setRotate(Color(0, 1, 0), yawAngle * PI / 180.0);
		gaze = yawMatrix * (pitchMatrix * Vector4(0.0, 0.0, 1.0, 0.0));
		viewup = yawMatrix * (pitchMatrix * Vector4(0.0, 1.0, 0.0, 0.0));
		cameraX = crossProduct(viewup, gaze);
		cameraX.normalize();
	}

	// ���������eyePos��gaze��viewup
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

	// ��y�������ƶ�
	void moveUp(const double speed) {
		eyePos.y += speed;
	}

	// gaze��XOZƽ���ϵ�ͶӰ��ǰ���ƶ�
	void moveForward(const double speed) {
		Vector4 forward(gaze.x, 0.0, gaze.z, 0.0);
		forward.normalize();
		eyePos += speed * forward;
	}

	// gaze��XOZƽ���ϵ�ͶӰ�������ƶ�
	void moveRight(const double speed) {
		Vector4 right(gaze.z, 0.0, -gaze.x, 0.0);
		right.normalize();
		eyePos += speed * right;
	}

	// ��������ת
	void pitch(const double theta) {
		Matrix rotate;
		rotate.setRotate(cameraX, theta);
		gaze = rotate * gaze;
		viewup = crossProduct(gaze, cameraX);
		gaze.normalize();
		viewup.normalize();
	}

	// ƫ������ת
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

// �ü���
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

// ����ǰ֡��view��project����
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

	// ����view
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

	// ����project
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

	// ����transform
	void updateTransform(const Camera& camera) {
		setView(camera);
		setProject(camera);
		transform = project * view;
	}

	const Matrix& getTransform() const { return transform; }
};

class Texture {
public:
	int32_t _w;			// ������
	int32_t _h;			// ����߶�
	BYTE** _bits;		// ��ɫ����
	// mipmap���
	std::vector<BYTE**> mipmap;		// mipmap������
	std::vector<int> mipmapWidths;	// �������и������
	std::vector<int> mipmapHeights;	// �������и����߶�
	double gamma = 1.1;				// ���mipmap������

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

	// ��ȡBMPͼƬ��Ϊ����
	void loadBmpTexture(const char* filename) {
		FILE* fp;
		errno_t err = fopen_s(&fp, filename, "rb");
		if (err) return;
		BITMAPINFOHEADER info;
		uint8_t header[14];
		int hr = (int)fread(header, 1, 14, fp);
		// fileheader��С������14�ֽ�
		if (hr != 14) { fclose(fp); return; }
		// �ļ���ʶ��������"BM"����0x424D����windowsλͼ�ļ�
		if (header[0] != 0x42 || header[1] != 0x4d) { fclose(fp); return; }
		hr = (int)fread(&info, 1, sizeof(info), fp);
		// infoheader��С������40�ֽ�
		if (hr != 40) { fclose(fp); return; }
		// ֻ֧��24/32λ���ָ�ʽ
		if (info.biBitCount != 24 && info.biBitCount != 32) { fclose(fp); return; }
		// ��ʼ��_bits
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
		// �õ�fileheader�е�ƫ��������ʼ��ȡ��������
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

	// ���ɺڰ׸�����
	void initBlackAndWhite() {
		// ��ʼ��_bits
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
		// �úڰ׸����_bits
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

	// ����texture����mipmap
	void initMipmap() {
		const int direction[] = { 0, 0, 1, 1, 0 };	// ������Ԫ����00 01 11 10����ʾ4������
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
					for (int i = 0; i < 4; ++i) {	// ȡ��һ��������4�����ص�ƽ��ֵ
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

	// �õ�ָ��level˫���Բ�ֵ��������ɫ�������ȴ���mipmap��
	Color getBilinearFilteredColor(const double iv, const double iu, const int ilevel = 0) {
		const int direction[] = { 0, 0, 1, 1, 0 };	// ������Ԫ����00 01 11 10����ʾ4������
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

	// �õ�ָ��level�������Բ�ֵ������ɫ�������ȴ���mipmap��
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
	int renderMode = 0;				// ��Ⱦģʽ��0-�߿� 1-������ɫ��� 2-������ͼ
	int renderModeCnt = 2;			// ��Ⱦģʽ������
	BYTE** framebuffer;				// ֡����
	double** zbuffer;				// ��Ȼ���
	int width, height;				// ��Ļ��Ⱥ���Ļ�߶�
	TransformMatrix tm;				// �任����
	Camera camera;					// �����
	Light light;					// ƽ�й�
	Texture texture;				// ֧��bmp��ȡ������ͼ

	RenderDevice(const int w, const int h, BYTE* screen_fb) : width(w), height(h) {
		// ��ʼ��framebuffer��ʹ���windows���ڵ�֡����
		framebuffer = new BYTE * [h];
		for (int i = 0; i < h; ++i)
			framebuffer[i] = screen_fb + 4 * w * i;
		// ��ʼ��zbuffer
		zbuffer = new double* [h];
		double* ptr = new double[w * h];
		for (int i = 0; i < h; ++i)
			zbuffer[i] = ptr + w * i;
		clearZbuffer();
		// ��ʼ�������
		camera.setCamera(Vector4(3.0, 3.0, -2.0, 1.0));
		// ��ʼ���任����
		tm.updateTransform(camera);
		// ��ʼ��ƽ�й�
		light = Light(Color(1.0, 1.0, 1.0), Vector4(-1.2, -0.9, 1.5, 0.0));
		// ���ɺڰ�����
		texture.initBlackAndWhite();
		// ����mipmap
		texture.initMipmap();
	}

	// ��ʾ��Ӧlevel��mipmap
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

	// �л���Ⱦģʽ
	void switchModeUp() {
		if (++renderMode == renderModeCnt)
			renderMode = 0;
	}

	void switchModeDown() {
		if (--renderMode < 0)
			renderMode = renderModeCnt - 1;
	}

	// ˢ��render device�ĸ���״̬
	void refresh(Color backgroundColor) {
		tm.updateTransform(camera);
		clearZbuffer();
		clearFramebuffer(backgroundColor);
	}

	// ���Ƴ���
	void drawScene(const Scene& scene) {
		for (const auto& mod : scene.models)
			drawModel(mod);
	}

	// ����ģ��
	void drawModel(const Model& model) {
		int faceSize = model.faces.size();
		vector<Triangle> willBeRendered;
		// ��װ�����Σ�����MVP�任���ü�������Ҫ��Ⱦ������������willBeRendered
		for (int i = 0; i < faceSize; ++i) {
			Triangle tri = assembleTriangleFromFace(model, i);	// ��faces����Ϣ����װ������
			triangleInitDiffuse(tri, light.direction);			// ���㶥���������ϵ��
			triangleModelTransform(tri, model.worldMatrix);		// model transform
			//if (renderMode != 0 && needFaceCulling(tri))		// �����޳�
			//	continue;
			triangleViewTransform(tri, model.worldMatrix);		// view transform
			triangleProjectTransform(tri, model.worldMatrix);	// project transform
			if (!triangleClipping(tri, willBeRendered))			// �ü�
				willBeRendered.emplace_back(tri);				// ��û���޳��������μӽ���Ⱦ����
		}
		// ��Ⱦ����������
		int renderSize = willBeRendered.size();
		for (int i = 0; i < renderSize; ++i) {
			triangleViewportTransform(willBeRendered[i]);			// �ӿڱ任
			triangleInitRhw(willBeRendered[i]);						// ����rhw��Ϊ͸�ӽ�����׼��
			if (renderMode == 0) {
				drawTriangleWireframe(willBeRendered[i]);
			} else {
				// ����������ε�mipLevel
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
				// ��ֳ��������ηֱ���Ⱦ
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

	// �����������������MipLevel
	double calculateMipLevel(const double triArea, const double textureArea) const {
		double mipLevel = log(sqrt(textureArea / triArea)) / log(2.0);
		mipLevel = clamp(mipLevel, 0.0, double(texture.mipmap.size() - 1));
		//printf("triArea = %.1lf, mipLevel = %.1lf\n", textureArea, triArea, mipLevel);
		return mipLevel;
	}

	// ��ʾ����
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
			// �ռ�������x������Ϣ
			if (tri.vertices[i].pos.x <= -tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_LX;
			else if (tri.vertices[i].pos.x >= tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_GX;
			else
				clipCodes[i] |= CLIP_CODE_IX;
			// �ռ�������y������Ϣ
			if (tri.vertices[i].pos.y <= -tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_LY;
			else if (tri.vertices[i].pos.y >= tri.vertices[i].pos.w)
				clipCodes[i] |= CLIP_CODE_GY;
			else
				clipCodes[i] |= CLIP_CODE_IY;
			// �ռ�������z������Ϣ��ֻ���ǽ�ƽ��Ĳü���������Զƽ�棩
			if (tri.vertices[i].pos.z < 0.0)
				clipCodes[i] |= CLIP_CODE_LZ;
			else
				clipCodes[i] |= CLIP_CODE_IZ;
		}

		// �ü�����������ͬʱ�����ҡ��ϡ��£�ƽ�����������
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

		// �����ƽ���ϵĲü�
		int outCnt = 0;	// ͳ���м��������ڽ�ƽ�����
		for (int i = 0; i < 3; ++i) {
			if ((clipCodes[i] & CLIP_CODE_LZ) == CLIP_CODE_LZ) {
				++outCnt;
			}
		}
		// 3�����㶼�ڽ�ƽ����棬ֱ�Ӷ���
		if (outCnt == 3)
			return true;
		// 3�����㶼�ڽ�ƽ��ǰ�棬ֱ�ӱ���
		else if (outCnt == 0)
			return false;
		// 2�������ڽ�ƽ����棬����ԭ�����Σ�Ϊ��Ⱦ���м���1��С������
		else if (outCnt == 2) {
			// ����������������
			Vertex inPoint, outPoint1, outPoint2;
			for (int i = 0; i < 3; ++i) {
				if ((clipCodes[i] & CLIP_CODE_IZ) == CLIP_CODE_IZ) {
					// ����˳���������������ݣ���֤���򲻱�
					inPoint = tri.vertices[i];
					outPoint1 = tri.vertices[(i + 1 + 3) % 3];
					outPoint2 = tri.vertices[(i + 2 + 3) % 3];
					break;
				}
			}
			double t1 = inPoint.pos.z / (inPoint.pos.z - outPoint1.pos.z);
			double t2 = inPoint.pos.z / (inPoint.pos.z - outPoint2.pos.z);
			// �߶�[inPoint, outPoint1]��[inPoint, outPoint2]�ͽ�ƽ��Ľ���
			Vertex intersection1 = interp(inPoint, outPoint1, t1);
			Vertex intersection2 = interp(inPoint, outPoint2, t2);
			// ����ü����С������
			willBeRendered.emplace_back(inPoint, intersection1, intersection2);
			return true;
			// 1�������ڽ�ƽ����棬����ԭ�����Σ�Ϊ��Ⱦ���м���2��С������
		} else if (outCnt == 1) {
			Vertex inPoint1, inPoint2, outPoint;
			for (int i = 0; i < 3; ++i) {
				if ((clipCodes[i] & CLIP_CODE_LZ) == CLIP_CODE_LZ) {
					// ����˳���������������ݣ���֤���򲻱�
					outPoint = tri.vertices[i];
					inPoint1 = tri.vertices[(i + 1 + 3) % 3];
					inPoint2 = tri.vertices[(i + 2 + 3) % 3];
					break;
				}
			}
			double t1 = inPoint1.pos.z / (inPoint1.pos.z - outPoint.pos.z);
			double t2 = inPoint2.pos.z / (inPoint2.pos.z - outPoint.pos.z);
			// �߶�[inPoint1, outPoint]��[inPoint2, outPoint]�ͽ�ƽ��Ľ���
			Vertex intersection1 = interp(inPoint1, outPoint, t1);
			Vertex intersection2 = interp(inPoint2, outPoint, t2);
			// ����ü����2��С������
			willBeRendered.emplace_back(inPoint1, inPoint2, intersection1);
			willBeRendered.emplace_back(intersection1, inPoint2, intersection2);
			return true;
		}
		return false;
	}

	// �ж��������Ƿ񱻲ü��������ַ���false�������
	// 1. �����κͽ�ƽ���ཻ
	// 2. �������������㶼���Ӿ�������
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

	// ���ظ��������Ƿ񱻱����޳�
	bool needFaceCulling(Triangle& tri) {
		tri.updateNormal();
		Vector4 pos = tri.vertices[0].pos;
		pos.divideW();
		return dotProduct(tri.normal, camera.getEyePos() - pos) <= 0.0;
	}

	// ���ݹ�����Ϣ���㶥���������ϵ��
	void triangleInitDiffuse(Triangle& tri, const Vector4& lightDirection) {
		tri.updateNormal();
		for (int i = 0; i < 3; ++i) {
			tri.vertices[i].diffuse = light.kd * (max(0.0, dotProduct(tri.normal, -light.direction)));
			//printf("%llf\n", tri.vertices[i].diffuse);
		}
	}

	// MVP�任
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

	// �ӿڱ任
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

	// ����rhw����׼��͸�ӽ���
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

	// �����������߿�Ĭ�Ϻ�ɫ
	void drawTriangleWireframe(const Triangle& triangle, const Color& color = Color::black()) {
		const Vertex& a = triangle.vertices[0], & b = triangle.vertices[1], & c = triangle.vertices[2];
		// �߿�ģʽ�����������
		drawLine(a.pos.y + 0.5, a.pos.x + 0.5, b.pos.y + 0.5, b.pos.x + 0.5, color);
		drawLine(b.pos.y + 0.5, b.pos.x + 0.5, c.pos.y + 0.5, c.pos.x + 0.5, color);
		drawLine(c.pos.y + 0.5, c.pos.x + 0.5, a.pos.y + 0.5, a.pos.x + 0.5, color);
	}

	// ��֡�������ֱ�ߣ���һ��ɫ��
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
			// ����Bresenham�㷨
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

	// �������ηֳ�1~2������
	vector<Trapzoid> splitTriangle(Triangle& tri) {
		Vertex& a = tri.vertices[0], & b = tri.vertices[1], & c = tri.vertices[2];
		// ����ʹ����������ϵ�����a,b,c
		if (a.pos.y > b.pos.y) std::swap(a, b);
		if (a.pos.y > c.pos.y) std::swap(a, c);
		if (b.pos.y > c.pos.y) std::swap(b, c);
		// ��������һ�ߵ����
		if (a.pos.y == b.pos.y && a.pos.y == c.pos.y) return vector<Trapzoid>();
		if (a.pos.x == b.pos.x && a.pos.x == c.pos.x) return vector<Trapzoid>();
		// 1�������ε����
		if (a.pos.y == b.pos.y) {
			if (a.pos.x > b.pos.x) std::swap(a, b);
			return vector<Trapzoid>{ Trapzoid{ a.pos.y, c.pos.y, a, b, c, c, tri.normal } };
		}
		if (b.pos.y == c.pos.y) {
			if (b.pos.x > c.pos.x) std::swap(b, c);
			return vector<Trapzoid>{ Trapzoid{ a.pos.y, b.pos.y, a, a, b, c, tri.normal } };
		}
		// 2�������ε����
		Trapzoid trap1, trap2;
		trap1.normal = trap2.normal = tri.normal;
		trap1.t = a.pos.y;
		trap1.b = b.pos.y;
		trap1.topL = trap1.topR = a;
		trap2.t = b.pos.y;
		trap2.b = c.pos.y;
		trap2.botL = trap2.botR = c;
		double k = (b.pos.y - a.pos.y) / (c.pos.y - a.pos.y);
		Vertex d = interp(a, c, k);	// �߶�bdΪ�ָ���
		if (b.pos.x > d.pos.x)
			std::swap(b, d);
		trap1.botL = trap2.topL = b;
		trap1.botR = trap2.topR = d;
		return vector<Trapzoid>{ trap1, trap2 };
	}

	// ɨ���߻�������
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

	// ����������y�����Ҷ˵����ɨ����
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

	// ��֡�����������
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

	// ��֡����������أ�zbuffer�㷨
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

	// ��color���֡����
	void clearFramebuffer(const Color& color) {
		for (int y = 0; y < height; ++y)
			for (int x = 0; x < width; ++x)
				drawPixel(y, x, color);
	}

	// �����Ȼ���
	void clearZbuffer() {
		for (int y = 0; y < height; ++y)
			for (int x = 0; x < width; ++x)
				zbuffer[y][x] = DBL_MAX;
	}
};