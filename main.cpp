#include <Novice.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <assert.h>
#include <imgui.h>
#include "Matrix4x4.h"
#include "Vector3.h"
#include "VectorMatrix.h"

using namespace std;

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

//球
struct Sphere
{
	Vector3 center;	//!<中心点
	float radius;	//!<半径
};

//直線
struct Line
{
	Vector3 origin;		//始点
	Vector3 diff;		//終点からの差分
};

//半直線
struct Ray
{
	Vector3 origin;		//始点
	Vector3 diff;		//終点からの差分
};

//線分
struct Segment
{
	Vector3 origin;		//始点
	Vector3 diff;		//終点からの差分
};

struct Plane
{
	Vector3 normal;		//!< 法線
	float distance;		//!< 距離
};

//Gridを表示する疑似コード
static void DrawGrid(const Matrix4x4& ViewProjectionMatrix, const Matrix4x4& ViewportMatrix)
{
	const float	kGridHalfWidth = 2.0f;										//Gridの半分の幅
	const uint32_t kSubdivision = 10;										//分割数
	const float kGridEvery = (kGridHalfWidth * 2.0f) / float(kSubdivision);	//1つ分の長さ

	//水平方向の線を描画
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; xIndex++)
	{
		//上の情報を使ってワールド座標系上の始点と終点を求める
		//X軸上の座標
		float posX = -kGridHalfWidth + kGridEvery * xIndex;

		//始点と終点
		Vector3 start = { posX, 0.0f, -kGridHalfWidth };
		Vector3 end = { posX, 0.0f, kGridHalfWidth };
		//// ワールド座標系 -> スクリーン座標系まで変換をかける
		start = Transform(start, Multiply(ViewProjectionMatrix, ViewportMatrix));
		end = Transform(end, Multiply(ViewProjectionMatrix, ViewportMatrix));

		//左から右も同じように順々に引いていく
		for (uint32_t zIndex = 0; zIndex <= kSubdivision; zIndex++)
		{
			//奥から手前が左右に代わるだけ
			//上の情報を使ってワールド座標系上の始点と終点を求める
			//Z軸上の座標
			float posZ = -kGridHalfWidth + kGridEvery * zIndex;

			//始点と終点
			Vector3 startZ = { -kGridHalfWidth, 0.0f, posZ };
			Vector3 endZ = { kGridHalfWidth, 0.0f, posZ };
			//// ワールド座標系 -> スクリーン座標系まで変換をかける
			startZ = Transform(startZ, Multiply(ViewProjectionMatrix, ViewportMatrix));
			endZ = Transform(endZ, Multiply(ViewProjectionMatrix, ViewportMatrix));

			//変換した画像を使って表示。色は薄い灰色(0xAAAAAAFF)、原点は黒ぐらいがいいが、なんでもいい
			Novice::DrawLine((int)start.x, (int)start.y, (int)end.x, (int)end.y, 0x6F6F6FFF);
			Novice::DrawLine((int)startZ.x, (int)startZ.y, (int)endZ.x, (int)endZ.y, 0x6F6F6FFF);
		}
	}
}

//Sphereを表示する疑似コード
static void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	const uint32_t kSubdivision = 20;							//分割数
	const float kLatStep = (float)M_PI / kSubdivision;			//緯度のステップ
	const float kLonStep = 2.0f * (float)M_PI / kSubdivision;	//経度のステップ

	// 緯度のループ
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex)
	{
		float lat = -0.5f * (float)M_PI + latIndex * kLatStep;	//現在の緯度

		//次の緯度
		float nextLat = lat + kLatStep;

		//経度のループ
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex)
		{
			//現在の経度
			float lon = lonIndex * kLonStep;

			//次の経度
			float nextLon = lon + kLonStep;

			// 球面座標の計算
			Vector3 pointA
			{
				sphere.center.x + sphere.radius * cos(lat) * cos(lon),
				sphere.center.y + sphere.radius * sin(lat),
				sphere.center.z + sphere.radius * cos(lat) * sin(lon)
			};

			Vector3 pointB
			{
				sphere.center.x + sphere.radius * cos(nextLat) * cos(lon),
				sphere.center.y + sphere.radius * sin(nextLat),
				sphere.center.z + sphere.radius * cos(nextLat) * sin(lon)
			};

			Vector3 pointC
			{
				sphere.center.x + sphere.radius * cos(lat) * cos(nextLon),
				sphere.center.y + sphere.radius * sin(lat),
				sphere.center.z + sphere.radius * cos(lat) * sin(nextLon)
			};

			// スクリーン座標に変換
			pointA = Transform(pointA, Multiply(viewProjectionMatrix, viewportMatrix));
			pointB = Transform(pointB, Multiply(viewProjectionMatrix, viewportMatrix));
			pointC = Transform(pointC, Multiply(viewProjectionMatrix, viewportMatrix));

			// 線分の描画
			Novice::DrawLine((int)pointA.x, (int)pointA.y, (int)pointB.x, (int)pointB.y, color);
			Novice::DrawLine((int)pointA.x, (int)pointA.y, (int)pointC.x, (int)pointC.y, color);
		}
	}
}

//球と球の衝突判定
bool IsCollision(const Sphere& s1, const Sphere& s2)
{
	//2つの球の中心点間の距離を求める
	float distance = Length(Subtract(s2.center, s1.center));
	// 半径の合計よりも短ければ衝突
	return distance <= (s1.radius + s2.radius);

}

//球と平面の衝突判定
bool IsCollisionSpherePlane(const Sphere& sphere, const Plane& plane)
{
	// 平面の法線ベクトルと球の中心点との距離
	float distance = Dot(plane.normal, sphere.center) - plane.distance;
	// その距離が球の半径以下なら衝突している
	return fabs(distance) <= sphere.radius;
}

Vector3 Perpendicular(const Vector3& vector)
{
	if (vector.x != 0.0f || vector.z != 0.0f)
	{
		return { -vector.z, 0.0f, vector.x }; // y軸以外の成分を使用
	}
	return { 0.0f, -vector.z, vector.y }; // y軸のみの場合
}

//平面の描画
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	Vector3 center = Multiply(plane.distance, plane.normal);

	Vector3 u = Normalize(Perpendicular(plane.normal));
	Vector3 v = Cross(plane.normal, u);

	// 平面の四隅を計算
	float extent = 10.0f; // 平面の描画範囲を設定 (適切な値に調整)
	Vector3 points[4];
	points[0] = Add(center, Add(Multiply(extent, u), Multiply(extent, v)));
	points[1] = Add(center, Add(Multiply(extent, u), Multiply(-extent, v)));
	points[2] = Add(center, Add(Multiply(-extent, u), Multiply(-extent, v)));
	points[3] = Add(center, Add(Multiply(-extent, u), Multiply(extent, v)));

	// 各頂点をビューポート座標に変換
	for (int32_t i = 0; i < 4; ++i) {　
		points[i] = Transform(Transform(points[i], viewProjectionMatrix), viewportMatrix);
	}

	// 平面を描画
	for (int32_t i = 0; i < 4; ++i) {
		Vector3 p1 = points[i];
		Vector3 p2 = points[(i + 1) % 4];
		Novice::DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, color);
	}
}

const char kWindowTitle[] = "LE2B_09_キクチ_ケンタ_提出用課題";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, kWindowWidth, kWindowHeight);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Segment segment{ {-2.0f,-1.0f,0.0f},{3.0f,2.0f,2.0f} };
	Vector3 point{ -1.5f,0.6f,0.6f };

	//各点の情報
	//Vector3 project = Project(Subtract(point, segment.origin), segment.diff);
	//Vector3 closestpoint = ClosestPoint(point, segment);

	//球体の情報
	Sphere sphere1{};
	Sphere sphere2{};

	//平面の情報
	Plane plane{};

	//Sphere pointSphere{ point,0.01f };		//1cmの球を描画
	//Sphere closestpointSphere{ closestpoint,0.01f };

	//SRTの情報
	Vector3 rotate = {};
	Vector3 translate = {};

	//カメラの位置
	Vector3 camaraTranslate = { 0.0f,1.9f,-6.49f };

	//カメラの角度
	Vector3 cameraRotate = { 0.26f,0.0f,0.0f };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		 // 2つの球の衝突判定
		//bool collision = IsCollision(sphere1, sphere2);

		bool collisionSpherePlane = IsCollisionSpherePlane(sphere1, plane);

		//各種行列の計算
		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, translate);
		Matrix4x4 viewWorldMatrix = Inverse(worldMatrix);

		//カメラの行列を作成
		Matrix4x4 cameraMatrxi = MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, camaraTranslate);
		//カメラのビュー行列
		Matrix4x4 viewCameraMatrix = Inverse(cameraMatrxi);

		// 透視投影行列を作成
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);

		//ビュー座標変換行列を作成
		Matrix4x4 ViewProjectionMatrix = Multiply(viewWorldMatrix, Multiply(viewCameraMatrix, projectionMatrix));

		//ViewportMatrixビューポート変換行列を作成
		Matrix4x4 ViewportMatrix = MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		Vector3 start = Transform(Transform(segment.origin, ViewProjectionMatrix), ViewportMatrix);
		Vector3 end = Transform(Transform(Add(segment.origin, segment.diff), ViewProjectionMatrix), ViewportMatrix);

		//bool collisionSpherePlane = IsCollisionSpherePlane(sphere1, plane);

		ImGui::Begin("Window");
		ImGui::DragFloat3("translate", &translate.x, 0.01f);
		ImGui::DragFloat3("CameraTranslate", &camaraTranslate.x, 0.01f);
		ImGui::DragFloat3("rotate", &rotate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &cameraRotate.x, 0.01f);
		ImGui::DragFloat3("SphereCenter1", &sphere1.center.x, 0.01f);
		ImGui::DragFloat("SphereRadius1", &sphere1.radius, 0.01f);
		ImGui::DragFloat3("Plane.Normal", &plane.normal.x, 0.01f);
		ImGui::DragFloat("Plane.Distance", &plane.distance, 0.01f);
		ImGui::End();

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		//Novice::DrawBox(0, 0, 1280, 720, 0.0f, BLACK, kFillModeSolid);

		// Gridを描画
		DrawGrid(ViewProjectionMatrix, ViewportMatrix);

		uint32_t spherePlaneColor = collisionSpherePlane ? RED : WHITE;

		// Sphereを描画
		//uint32_t sphere1Color = collisionSpherePlane ? RED : WHITE;

		DrawSphere(sphere1, ViewProjectionMatrix, ViewportMatrix, spherePlaneColor);
		DrawPlane(plane, ViewProjectionMatrix, ViewportMatrix, 0x6F6F6FFF);
		Novice::DrawLine((int)start.x, (int)start.y, (int)end.x, (int)end.y, 0x00000000);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの
	Novice::Finalize();
	return 0;
}
