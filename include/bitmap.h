#if !defined(BITMAP_H)
#define BITMAP_H

#include <cstdint>
#include <string>
#include <vector>

#include "vector.h"

struct BitmapMagic {
	uint8_t 	magic[2];
};

struct BitmapFileHeader {
	uint32_t 	size;
	uint16_t 	reserved1;
	uint16_t 	reserved2;
	uint32_t 	offset;
};

struct BitmapInfoHeader {
	uint32_t	size;
	int32_t		width;
	int32_t 	height;
	uint16_t 	numPlanes;
	uint16_t	bitsPerPixel;
	uint32_t 	compressionType;
	uint32_t 	bitmapSize;
	int32_t 	xResolution;
	int32_t 	yResolution;
	uint32_t 	numColors;
	uint32_t 	numImportantColors;
};

class Bitmap {
public:
	Bitmap(int32_t width, int32_t height)
		: width_(width)
		, height_(height)
	{
		buffer_.resize(width_ * height_);
	}

	void set(int32_t x, int32_t y, const Vector3f& color)
	{
		buffer_[y * width_ + x] = color;
	}

	bool write(const std::string& name);

private:
	int32_t width_;
	int32_t height_;

	std::vector<Vector3f> buffer_;
};

#endif // BITMAP_H
