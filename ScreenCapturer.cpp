#include "ScreenCapturer.h"
#include "lib/natives.h"
#include "defaults.h"
#include <stdlib.h>
#include <stdio.h>


ScreenCapturer::ScreenCapturer() : pixels(NULL), isActivate(false){

}

ScreenCapturer::~ScreenCapturer() {
	DestroyCapturer();
}

void ScreenCapturer::setDatasetParams(const Value & dc, Document & d)
{
	if (!dc["frame"].IsNull()) {
		DestroyCapturer();

		if (!dc["frame"][0].IsNull()) imageWidth = dc["frame"][0].GetInt();
		else imageWidth = _WIDTH_;

		if (!dc["frame"][1].IsNull()) imageHeight = dc["frame"][1].GetInt();
		else imageHeight = _HEIGHT_;

		isActivate = true;
		// Round up the scan line size to a multiple of 4
		length = ((imageWidth * 3 + 3) / 4 * 4) * imageHeight;

		//Screen capture buffer
		GRAPHICS::_GET_SCREEN_ACTIVE_RESOLUTION(&windowWidth, &windowHeight);
		hWnd = ::FindWindow(NULL, "Grand Theft Auto V");
		//hWindowDC = GetDC(NULL);
		hWindowDC = GetDC(hWnd);
		//the capture tool's object with memory
		hCaptureDC = CreateCompatibleDC(hWindowDC);
		//the bitmap format object
		hCaptureBitmap = CreateCompatibleBitmap(hWindowDC, imageWidth, imageHeight);
		SelectObject(hCaptureDC, hCaptureBitmap);
		SetStretchBltMode(hCaptureDC, COLORONCOLOR);

		pixels = (UINT8*)malloc(length);
		info.biSize = sizeof(BITMAPINFOHEADER);
		info.biPlanes = 1;
		info.biBitCount = 24;
		info.biWidth = imageWidth;
		info.biHeight = -imageHeight;
		info.biCompression = BI_RGB;
		info.biSizeImage = 0;
	}
	else DestroyCapturer();
}

void ScreenCapturer::DestroyCapturer()
{
	if (!isActivate) return;

	if (pixels != NULL) {
		free(pixels);
		pixels = NULL;
	}
	isActivate = false;
	ReleaseDC(hWnd, hWindowDC);
	DeleteDC(hCaptureDC);
	DeleteObject(hCaptureBitmap);
}

bool ScreenCapturer::isCapturerActivate()
{
	return isActivate;
}

void ScreenCapturer::capture() {
	//time cost is about 15~32ms
	// copies a bitmap from a source rectangle into a destination rectangle, stretching or compressing the bitmap to fit
	switch (isActivate)
	{
	case true:
		StretchBlt(hCaptureDC, 0, 0, imageWidth, imageHeight, hWindowDC, 0, 0, windowWidth, windowHeight, SRCCOPY);
		GetDIBits(hCaptureDC, hCaptureBitmap, 0, imageHeight, pixels, (BITMAPINFO*)&info, DIB_RGB_COLORS);
		break;
	}
}