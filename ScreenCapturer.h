#pragma once

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include "Task.hpp"

class ScreenCapturer : public ISetDatasetParams
{
private:
	int windowWidth;
	int windowHeight;

	HWND hWnd;
	HDC hWindowDC;
	HDC hCaptureDC;
	HBITMAP hCaptureBitmap;
	BITMAPINFOHEADER info;

	unsigned int length;
	UINT8* pixels;

	bool isActivate;
public:
	friend class Server;
	int imageWidth;
	int imageHeight;
	void setDatasetParams(const Value& dc, Document &d);
	void DestroyCapturer();
	bool isCapturerActivate();
	ScreenCapturer();
	~ScreenCapturer();

	void capture();
	
};