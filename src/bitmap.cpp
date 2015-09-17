#include "bitmap.h"

#include <cstdint>
#include <algorithm>
#include <fstream>

static uint8_t toneMap(float val)
{
	static const uint8_t max = 255;
	return std::min(max, (uint8_t)(std::pow(1 - std::exp(-val), 1.0f / 2.2f) * 255 + 0.5f));
}

bool Bitmap::write(const std::string& name)
{
	bool success = false;

	int32_t rowSize = ((24 * width_ + 31) / 32) * 4;
	int32_t paddingSize = rowSize - (width_ * 3);
	uint8_t padding[] = { 0, 0, 0 };

	std::ofstream out(name, std::ios::out | std::ios::binary);

	BitmapMagic magic = { { 'B', 'M' } };

	BitmapFileHeader fileHeader;
	fileHeader.size = sizeof(BitmapMagic) + sizeof(BitmapFileHeader)
		+ sizeof(BitmapInfoHeader) + rowSize * height_;
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
	infoHeader.bitmapSize = rowSize * height_;
	infoHeader.xResolution = 0;
	infoHeader.yResolution = 0;
	infoHeader.numColors = 0;
	infoHeader.numImportantColors = 0;

	out.write(reinterpret_cast<char*>(&magic), sizeof(BitmapMagic));

	out.write(reinterpret_cast<char*>(&fileHeader), sizeof(BitmapFileHeader));

	out.write(reinterpret_cast<char*>(&infoHeader), sizeof(BitmapInfoHeader));

	uint8_t color[3];

	for (int32_t i = height_ - 1; i >= 0; --i) {
		for (int32_t j = 0; j < width_; ++j) {
			const Vector3f& bufVal = buffer_[i * width_ + j];
			color[0] = toneMap(bufVal.z);
			color[1] = toneMap(bufVal.y);
			color[2] = toneMap(bufVal.x);

			out.write(reinterpret_cast<char*>(color), sizeof(color));
		}
		if (padding > 0) {
			out.write(reinterpret_cast<char*>(padding), paddingSize);
		}
	}

	success = true;

	return success;
}
