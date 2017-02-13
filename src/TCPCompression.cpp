// Avisynth v2.5.  Copyright 2002 Ben Rudiak-Gould et al.
// Avisynth v2.6.  Copyright 2006 Klaus Post.
// Avisynth v2.6.  Copyright 2009 Ian Brabham.
// Avisynth+ project
// Last modification date: 20170117

// http://www.avisynth.org

// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .
//
// Linking Avisynth statically or dynamically with other modules is making
// a combined work based on Avisynth.  Thus, the terms and conditions of
// the GNU General Public License cover the whole combination.
//
// As a special exception, the copyright holders of Avisynth give you
// permission to link Avisynth with independent modules that communicate
// with Avisynth solely through the interfaces defined in avisynth.h,
// regardless of the license terms of these independent modules, and to
// copy and distribute the resulting combined work under terms of your
// choice, provided that every copy of the combined work is accompanied
// by a complete copy of the source code of Avisynth (the version of
// Avisynth used to produce the combined work), being distributed under
// the terms of the GNU General Public License plus this exception.  An
// independent module is a module which is not derived from or based on
// Avisynth, such as 3rd-party filters, import and export plugins, or
// graphical user interfaces.

// TCPDeliver (c) 2004 by Klaus Post, (c) 2017 DJATOM

#include <TCPCompression.h>

/******************************
 * DeltaEncode and DeltaDecode methods
 * Rewrite from inline asm by pinterf
 * Some additions by DJATOM
 ******************************/

template<typename pixel_t>
void TCPCompression::DeltaEncode(uint8_t* image, int pitch, int rowsize, int h) {
    for (int x = 0; x < rowsize; x += 16)
    {
        uint8_t* pdst = image;
        __m128i top = _mm_setzero_si128(); // current top, initially zero
        for (int y = 0; y < h; y++)
        {
            __m128i temp = _mm_load_si128(reinterpret_cast<const __m128i*>(pdst + x)); // or loadu (unaligned)
            __m128i result;
            
            if (sizeof(pixel_t) == 1)
                result = _mm_sub_epi8(temp, top); // 8 bit subtract, no saturation
            else if (sizeof(pixel_t) == 2)
                result = _mm_sub_epi16(temp, top); // 16 bit subtract, no saturation
            else
                result = _mm_sub_epi32(temp, top); // 32 bit subtract, no saturation
            _mm_store_si128(reinterpret_cast<__m128i*>(pdst + x), result); // or storeu if unaligned
            top = temp; // top = temp
            pdst += pitch; // next line
        }
    }
}

template<typename pixel_t>
void TCPCompression::DeltaDecode(uint8_t* image, int pitch, int rowsize, int h) {
    for (int x = 0; x < rowsize; x += 16)
    {
        uint8_t* pdst = image;
        __m128i top = _mm_setzero_si128(); // current left, initially zero
        for (int y = 0; y < h; y++)
        {
            __m128i src = _mm_load_si128(reinterpret_cast<const __m128i*>(pdst + x));
            if (sizeof(pixel_t) == 1)
                top = _mm_add_epi8(top, src); // 8 bit add, no saturation
            else if (sizeof(pixel_t) == 2)
                top = _mm_add_epi16(top, src); // 16 bit add, no saturation
            else
                top = _mm_add_epi32(top, src); // 32 bit add, no saturation
            _mm_store_si128(reinterpret_cast<__m128i*>(pdst + x), top);
            pdst += pitch; // next line
        }
    }
}

/******************************
 * Default scheme (no compression)
 ******************************/

int TCPCompression::CompressImage(uint8_t* image, int rowsize, int h, int pitch) {
    dst = image;
    inplace = true;
    return pitch*h;
}

int TCPCompression::DeCompressImage(uint8_t* image, int rowsize, int h, int pitch, int data_size) {
    dst = image;
    inplace = true;
    return pitch*h;
}

/******************************
* LZO compression scheme.
******************************/

PredictDownLZO::PredictDownLZO() {
    compression_type = ServerFrameInfo::COMPRESSION_DELTADOWN_LZO;
    wrkmem = (lzo_bytep) malloc(LZO1X_1_MEM_COMPRESS);
}

PredictDownLZO::~PredictDownLZO(void) {
    free(wrkmem);
}

int PredictDownLZO::CompressImage(uint8_t* image, int rowsize, int h, int pitch) {
    // Pitch mod 16
    // Height > 2
    rowsize = (rowsize+15)&~15;
    inplace = false;

    switch (componentsize) {
    case 1:
        DeltaEncode<uint8_t>(image, pitch, rowsize, h);
        break;
    case 2:
        DeltaEncode<uint16_t>(image, pitch, rowsize, h);
        break;
    case 4:
        DeltaEncode<float>(image, pitch, rowsize, h);
        break;
    default:
        DeltaEncode<uint8_t>(image, pitch, rowsize, h);
        break;
    }

    int in_size = pitch*h;
    int out_size = -1;
    dst = (uint8_t*)_aligned_malloc(in_size + (in_size >>6) + 16 + 3, 16);
    lzo1x_1_compress(image, in_size ,(unsigned char *)dst, (lzo_uint *)&out_size, wrkmem);
    _RPT2(0, "TCPCompression: Compressed %d bytes into %d bytes (LZO).\n", in_size, out_size);
    return    out_size;
}
 
int PredictDownLZO::DeCompressImage(uint8_t* image, int rowsize, int h, int pitch, int in_size) {
    // Pitch mod 16
    // Height > 2
    inplace = false;        
    lzo_uint dst_size = pitch*h;
    dst = (uint8_t*)_aligned_malloc(dst_size, 64);
    lzo1x_decompress(image, in_size, dst, &dst_size, wrkmem);

    if ((int)dst_size != pitch*h) {
        _RPT0(1, "TCPCompression: Size did NOT match");
    }

    rowsize = (rowsize+15)&~15;
    image = dst;

    switch (componentsize) {
    case 1:
        DeltaDecode<uint8_t>(image, pitch, rowsize, h);
        break;
    case 2:
        DeltaDecode<uint16_t>(image, pitch, rowsize, h);
        break;
    case 4:
        DeltaDecode<float>(image, pitch, rowsize, h);
        break;
    default:
        DeltaDecode<uint8_t>(image, pitch, rowsize, h);
        break;
    }

    _RPT2(0, "TCPCompression: Decompressed %d bytes into %d bytes (LZO).\n", in_size, dst_size);
    return (int)dst_size;
}

/******************************
* Huffman compression scheme.
******************************/

PredictDownHuffman::PredictDownHuffman() {
    compression_type = ServerFrameInfo::COMPRESSION_DELTADOWN_HUFFMAN;
}

PredictDownHuffman::~PredictDownHuffman(void) {
}

int PredictDownHuffman::CompressImage(uint8_t* image, int rowsize, int h, int pitch) {
    // Pitch mod 16
    // Height > 2
    inplace = false;
    rowsize = (rowsize+15)&~15;

    switch (componentsize) {
    case 1:
        DeltaEncode<uint8_t>(image, pitch, rowsize, h);
        break;
    case 2:
        DeltaEncode<uint16_t>(image, pitch, rowsize, h);
        break;
    case 4:
        DeltaEncode<float>(image, pitch, rowsize, h);
        break;
    default:
        DeltaEncode<uint8_t>(image, pitch, rowsize, h);
        break;
    }

    int in_size = pitch*h;
    unsigned int out_size = in_size*2;
    dst = (uint8_t*)_aligned_malloc(out_size, 16);
    out_size = Huffman_Compress(image, dst, in_size );

    _RPT2(0, "TCPCompression: Compressed %d bytes into %d bytes (Huffman).\n", in_size, out_size);
    return out_size;
}
 
int PredictDownHuffman::DeCompressImage(uint8_t* image, int rowsize, int h, int pitch, int in_size) {
    // Pitch mod 16
    // Height > 2
    inplace = false;
    unsigned int dst_size = pitch*h;
    dst = (uint8_t*)_aligned_malloc(dst_size, 64);

    Huffman_Uncompress(image, dst, in_size, dst_size);

    rowsize = (rowsize+15)&~15;
    image = dst;
    
    switch (componentsize) {
    case 1:
        DeltaDecode<uint8_t>(image, pitch, rowsize, h);
        break;
    case 2:
        DeltaDecode<uint16_t>(image, pitch, rowsize, h);
        break;
    case 4:
        DeltaDecode<float>(image, pitch, rowsize, h);
        break;
    default:
        DeltaDecode<uint8_t>(image, pitch, rowsize, h);
        break;
    }

    _RPT2(0, "TCPCompression: Decompressed %d bytes into %d bytes (Huffman).\n", in_size, dst_size);
    return dst_size;
}

/******************************
* GZip compression scheme.
******************************/

PredictDownGZip::PredictDownGZip() {
    compression_type = ServerFrameInfo::COMPRESSION_DELTADOWN_GZIP;
    z = (z_stream_s*)malloc(sizeof(z_stream_s));
}

PredictDownGZip::~PredictDownGZip(void) {
    free(z);
}

int PredictDownGZip::CompressImage(uint8_t* image, int rowsize, int h, int pitch) {
    // Pitch mod 16
    // Height > 2
    inplace = false;
    rowsize = (rowsize+15)&~15;
        
    switch (componentsize) {
    case 1:
        DeltaEncode<uint8_t>(image, pitch, rowsize, h);
        break;
    case 2:
        DeltaEncode<uint16_t>(image, pitch, rowsize, h);
        break;
    case 4:
        DeltaEncode<float>(image, pitch, rowsize, h);
        break;
    default:
        DeltaEncode<uint8_t>(image, pitch, rowsize, h);
        break;
    }

    int in_size = pitch*h;
    unsigned int out_size = in_size*2;
    dst = (uint8_t*)_aligned_malloc(out_size, 16);
    memset(z, 0, sizeof(z_stream_s));

    z->avail_in = in_size;
    z->next_in = image;
    z->total_in = 0;

    z->avail_out = out_size;
    z->next_out = dst;
    z->total_out = 0;
    
    z->data_type = Z_BINARY;
    deflateInit2(z, Z_BEST_SPEED, Z_DEFLATED, 15, 8, Z_HUFFMAN_ONLY);
    int i = deflate(z, Z_FINISH);

    deflateEnd(z);
    out_size = z->total_out;
    unsigned int* dstint = (unsigned int*)&dst[out_size];
    dstint[0]= z->adler;

    out_size+=4;
    _RPT2(0, "TCPCompression: Compressed %d bytes into %d bytes (GZip).\n", in_size, out_size);
    return out_size;
}
 
int PredictDownGZip::DeCompressImage(uint8_t* image, int rowsize, int h, int pitch, int in_size) {
    // Pitch mod 16
    // Height > 2
    inplace = false;
    unsigned int dst_size = pitch*h;
    dst = (uint8_t*)_aligned_malloc(dst_size, 64);
    memset(z, 0, sizeof(z_stream_s));

    unsigned int* dstint = (unsigned int*)&image[in_size-4];
    z->adler = dstint[0]; 
    in_size-=4;

    z->avail_in = in_size;
    z->next_in = image;
    z->total_in = 0;

    z->avail_out = dst_size;
    z->next_out = dst;
    z->total_out = 0;

    z->data_type = Z_BINARY;

    inflateInit(z);
    int i = inflate(z, Z_FINISH);
    inflateEnd(z);

    rowsize = (rowsize+15)&~15;
    image = dst;

    switch (componentsize) {
    case 1:
        DeltaDecode<uint8_t>(image, pitch, rowsize, h);
        break;
    case 2:
        DeltaDecode<uint16_t>(image, pitch, rowsize, h);
        break;
    case 4:
        DeltaDecode<float>(image, pitch, rowsize, h);
        break;
    default:
        DeltaDecode<uint8_t>(image, pitch, rowsize, h);
        break;
    }

    _RPT2(0, "TCPCompression: Decompressed %d bytes into %d bytes (GZip).\n", in_size, dst_size);
    return dst_size;
}
