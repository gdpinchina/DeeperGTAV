#pragma once
#include <Eigen/Core>

typedef struct _native_buff
{
	long len;
	void *p;
} native_buff;

struct rage_matrices
{
	Eigen::Matrix4f world;
	Eigen::Matrix4f worldView;
	Eigen::Matrix4f worldViewProjection;
	Eigen::Matrix4f invView;
};

typedef int(*pInterface_1)(void **);
typedef int(*pInterface_2)(rage_matrices *);
typedef long(*pInterface_3)();


class VisionNative
{
protected:
	VisionNative();
	~VisionNative();

public:
	static VisionNative* GetInstance();
	static void DestroyInstance();
	void loadLib();//if loaded, it won't be loaded again.
	void unLoadLib();
	native_buff const* GetDepthBuffer();
	native_buff const* GetFrontBuffer();
	native_buff const* GetStencilBuffer();
	rage_matrices const* GetConstants();
	long GetLastDepthTime();
	long GetLastColorTime();
	long GetLastConstantTime();
	long GetPresentTime();

private:
	HINSTANCE hVN;

	pInterface_1 _export_get_depth_buffer;
	pInterface_1 _export_get_color_buffer;
	pInterface_1 _export_get_stencil_buffer;
	pInterface_2 _export_get_constant_buffer;
	pInterface_3 _export_get_last_depth_time;
	pInterface_3 _export_get_last_color_time;
	pInterface_3 _export_get_last_constant_time;
	pInterface_3 _export_get_current_time;

	native_buff _depth_buffer;
	native_buff _color_buffer;
	native_buff _stencil_buffer;
	rage_matrices _constant_buffer;

	static VisionNative* _instance;
};

