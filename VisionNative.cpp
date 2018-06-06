#include<Windows.h>
#include<stdio.h>
#include "VisionNative.h"

VisionNative* VisionNative::_instance = NULL;

VisionNative::VisionNative()
{
	hVN = NULL;
	_depth_buffer.p = NULL;
	_color_buffer.p = NULL;
	_stencil_buffer.p = NULL;
}

VisionNative::~VisionNative()
{
	unLoadLib();
	if (_depth_buffer.p)
	{
		free(_depth_buffer.p);
		_depth_buffer.p = NULL;
	}
	if(_color_buffer.p != NULL)
	{
		free(_color_buffer.p);
		_color_buffer.p = NULL;
	}

	if(_stencil_buffer.p != NULL)
	{
		free(_stencil_buffer.p);
		_stencil_buffer.p = NULL;
	}
}

VisionNative * VisionNative::GetInstance()
{
	if (_instance == NULL)
		_instance = new VisionNative();
	return _instance;
}

void VisionNative::DestroyInstance()
{
	if (_instance != NULL)
	{
		delete(_instance);
		_instance = NULL;
	}
}

void VisionNative::loadLib()
{
	if (hVN == NULL)
	{
		hVN = LoadLibrary("./GTAVisionNative.asi");

		_export_get_depth_buffer = (pInterface_1)GetProcAddress(hVN, "export_get_depth_buffer");
		_export_get_color_buffer = (pInterface_1)GetProcAddress(hVN, "export_get_color_buffer");
		_export_get_stencil_buffer = (pInterface_1)GetProcAddress(hVN, "export_get_stencil_buffer");
		_export_get_constant_buffer = (pInterface_2)GetProcAddress(hVN, "export_get_constant_buffer");
		_export_get_last_depth_time = (pInterface_3)GetProcAddress(hVN, "export_get_last_depth_time");
		_export_get_last_color_time = (pInterface_3)GetProcAddress(hVN, "export_get_last_color_time");
		_export_get_last_constant_time = (pInterface_3)GetProcAddress(hVN, "export_get_last_constant_time");
		_export_get_current_time = (pInterface_3)GetProcAddress(hVN, "export_get_current_time");

		if (_export_get_depth_buffer && _export_get_color_buffer && _export_get_stencil_buffer && _export_get_constant_buffer &&
			_export_get_last_depth_time && _export_get_last_color_time && _export_get_last_constant_time && _export_get_current_time)
			printf("\nLoad asi successfully");
		else
			printf("\nLoad asi failed");
	}
}

void VisionNative::unLoadLib()
{
	if (hVN != NULL)
	{
		FreeLibrary(hVN);
		hVN = NULL;
		printf("\nFree asi successfully");
	}
}

native_buff const* VisionNative::GetDepthBuffer()
{
	if (hVN != NULL)
	{
		void *pBuf;
		long sz = _export_get_depth_buffer(&pBuf);
		if (sz == -1) return NULL;
		if (sz > _depth_buffer.len)
		{
			_depth_buffer.p = (void *)realloc(_depth_buffer.p, sz);
			_depth_buffer.len = sz;
		}
		memcpy(_depth_buffer.p, pBuf, _depth_buffer.len);
		return &_depth_buffer;
	}
	else
		return NULL;
}

native_buff const* VisionNative::GetFrontBuffer()
{
	if (hVN != NULL)
	{
		void *pBuf;
		long sz = _export_get_color_buffer(&pBuf);
		if (sz == -1) return NULL;
		if (sz > _color_buffer.len)
		{
			_color_buffer.p = (void *)realloc(_color_buffer.p, sz);
			_color_buffer.len = sz;
		}
		memcpy(_color_buffer.p, pBuf, _color_buffer.len);
		return &_color_buffer;
	}
	else
		return NULL;
}

native_buff const* VisionNative::GetStencilBuffer()
{
	if (hVN != NULL)
	{
		void *pBuf;
		long sz = _export_get_stencil_buffer(&pBuf);
		if (sz == -1) return NULL;
		if (sz > _stencil_buffer.len)
		{
			_stencil_buffer.p = (void *)realloc(_stencil_buffer.p, sz);
			_stencil_buffer.len = sz;
		}
		memcpy(_stencil_buffer.p, pBuf, _stencil_buffer.len);
		return &_stencil_buffer;
	}
	else
		return NULL;
}

rage_matrices const* VisionNative::GetConstants()
{
	if (hVN != NULL)
	{
		long sz = _export_get_constant_buffer(&_constant_buffer);
		if (sz != sizeof(_constant_buffer)) return NULL;
		return &_constant_buffer;
	}
	else
		return NULL;
}

long VisionNative::GetLastDepthTime()
{
	if (hVN != NULL)
		return _export_get_last_depth_time();
	else
		return NULL;
}

long VisionNative::GetLastColorTime()
{
	if (hVN != NULL)
		return _export_get_last_color_time();
	else
		return NULL;
}

long VisionNative::GetLastConstantTime()
{
	if (hVN != NULL)
		return _export_get_last_constant_time();
	else
		return NULL;
}

long VisionNative::GetPresentTime()
{
	if (hVN != NULL)
		return _export_get_current_time();
	else
		return NULL;
}

