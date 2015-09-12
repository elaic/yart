#include "bitmap.h"

#include <cstdint>
#include <algorithm>
#include <fstream>

static uint8_t toneMap(float val)
{
	return static_cast<uint8_t>(
		std::min(255.0f, std::pow(1 - std::exp(-val), 1.0f / 2.2f) * 255 + 0.5f));
}

bool Bitmap::write(const std::string& name)
{
	bool success = false;

	int32_t rowSize = std::floor((24 * width_ + 31) / 32) * 4;

	std::ofstream out(name);

	BitmapMagic magic = { { 'B', 'M' } };

	BitmapFileHeader fileHeader;
	fileHeader.size = sizeof(BitmapMagic) + sizeof(BitmapFileHeader)
		+ sizeof(BitmapInfoHeader) + width_ * height_ * 3;
	fileHeader.reserved1 = 0;
	fileHeader.reserved2 = 0;
	fileHeader.offset = sizeof(BitmapMagic) + sizeof(BitmapFileHeader)
		+ sizeof(BitmapInfoHeader);

	BitmapInfoHeader infoHeader;
	infoHeader.size = sizeof(BitmapInfoHeader);
	infoHeader.width = width_;
	infoHeader.height = height_;
	infoHeader.numPlanes = 1;
	infoHeader.bitsPerPixel = 24;
	infoHeader.compressionType = 0;
	infoHeader.bitmapSize = width_ * height_ * 3;
	infoHeader.xResolution = 0x0ec4;
	infoHeader.yResolution = 0x0ec4;
	infoHeader.numColors = 0;
	infoHeader.numImportantColors = 0;

	out.write(reinterpret_cast<char*>(&magic), sizeof(BitmapMagic));

	out.write(reinterpret_cast<char*>(&fileHeader), sizeof(BitmapFileHeader));

	out.write(reinterpret_cast<char*>(&infoHeader), sizeof(BitmapInfoHeader));

	for (int32_t i = height_ - 1; i >= 0; --i) {
		for (int32_t j = 0; j < width_; ++j) {
			const Vector3f& bufVal = buffer_[i * width_ + j];
			uint8_t color[3];
			color[0] = toneMap(bufVal.z);
			color[1] = toneMap(bufVal.y);
			color[2] = toneMap(bufVal.x);

			out.write(reinterpret_cast<char*>(color), sizeof(color));
		}
		// use row size to add padding if necessary
	}

	success = true;

	return success;
}
