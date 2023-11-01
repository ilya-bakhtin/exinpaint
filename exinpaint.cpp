/*
    ExInpaint Filter for Avisynth 2.5 -  Exemplar-Based Inpainting
	Copyright (C) 2008 A.G.Balakhnin aka Fizick, http://avisynth.org.ru

    This program is free software; you can rrdistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

*/

#include "windows.h"
#include "avisynth.h"
#include "inpainting.h"

//-------------------------------------------------------------------------------------------
class ExInpaint : public GenericVideoFilter {

	//  parameters
	PClip maskclip;
	int color;
	int dilate;
	int xsize;
	int ysize;
	int radius;
	int maxsteps;

	inpainting *inp;
	unsigned char * bufferYUV;
	unsigned char * buffermaskYUV; // mask
	int buffer_pitch;

public:

	ExInpaint(PClip _child,  PClip _maskclip, int _color, int _dilate, int _xsize, int _ysize, int _radius, int _maxsteps, IScriptEnvironment* env);
  ~ExInpaint();
	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
};


//Here is the acutal constructor code used
ExInpaint::ExInpaint(PClip _child, PClip _maskclip, int _color, int _dilate, int _xsize, int _ysize, int _radius, int _maxsteps, IScriptEnvironment* env):

	GenericVideoFilter(_child), maskclip(_maskclip), color(_color), dilate(_dilate), xsize(_xsize), ysize(_ysize), radius(_radius), maxsteps(_maxsteps) {
  // This is the implementation of the constructor.
  // The child clip (source clip) is inherited by the GenericVideoFilter,
  //  where the following variables gets defined:
  //   PClip child;   // Contains the source clip.
  //   VideoInfo vi;  // Contains videoinfo on the source clip.

	int pixel_format;

    if (maskclip == 0) // no mask clip
    {
        if ( vi.IsRGB32() )
            pixel_format = RGBA; // use Alpha channel as a mask
        else
            env->ThrowError("ExInpaint: without mask clip video must be RGB32!");
    }
    else // have mask clip
    {
        if ( vi.IsRGB32() )
            pixel_format = RGB32; // but do not use Alpha channel
        else if ( vi.IsRGB24() )
            pixel_format = RGB24;
        else if ( vi.IsYV12() )
            pixel_format = YV12;
        else if ( vi.IsYUY2() )
		{
            pixel_format = YUV24;
			buffer_pitch = (((vi.width+3)/4)*4)*3; // mult 4
			bufferYUV = new unsigned char [vi.height * buffer_pitch+16];
			buffermaskYUV = new unsigned char [vi.height * buffer_pitch+16];
		}
        else
            env->ThrowError("ExInpaint: video must be RGB32 or RGB24 or YV12 or YUY2!");

        VideoInfo maskvi = maskclip->GetVideoInfo();

        if (vi.width != maskvi.width || vi.height != maskvi.height )
            env->ThrowError("ExInpaint: mask size %d x %d is not as source clip size",maskvi.width,maskvi.height);

        if (vi.pixel_type != maskvi.pixel_type)
            env->ThrowError("ExInpaint: mask pixel type must be same as source clip type!");
    }

//	masklast = maskvi.num_frames;

// warning:
//    if (!(xsize%2 && ysize%2))
//		env->ThrowError("ExInpaint: xsize, ysize must be odd (3,5,7,9...)!");

	inp = new inpainting(vi.width, vi.height, pixel_format);

}
//-------------------------------------------------------------------------------------------

// This is where any actual destructor code used goes
ExInpaint::~ExInpaint() {
	delete inp;
	delete [] bufferYUV;
	delete [] buffermaskYUV;
}


void convertYUY2toYUV24(const BYTE * psrcYUY2, int src_pitch, int row_size, int src_height,
			BYTE * bufYUV24, int buf_pitch)
{ // convert YUY2 to interleaved not-subsampled YUV24
#if (ISSE)

	_asm {
		//push rsi;
		//push rdi;
		//push rbx;
		mov ecx, src_height;
		mov eax, row_size; // double width
		shr rax, 1; // width in pixels
		mov ebx, src_pitch;
		mov edx, buf_pitch;
		mov rsi, psrcYUY2;
		mov rdi, bufYUV24;
		pxor mm7,mm7; // 0
		//push rbp;
nexth:
		jecxz endh;

		xor rbp, rbp; // 0
nextw:
		cmp rbp, rax;
		jge endw;
		movd mm0, [rsi]; // 2 pixels V Y2 U Y1
		punpcklbw mm0, mm7;
		movq mm1, mm0;
		pshufw mm0,mm0, 10110100b; //  Y2 V U Y1
		packuswb mm0, mm0;
		pshufw mm1,mm1, 11011101b; //  V U V U
		packuswb mm1, mm1;
		punpckldq mm0, mm1; // V U V U Y2 V U Y1    high two bytes is not used
		movq [rdi], mm0;
		add rsi, 4;
		add rdi, 6;
		add rbp, 2;
		jmp nextw;
endw:
		sub rsi, rbp;
		sub rsi, rbp;
		add rsi, rbx; // pitch

		sub rdi, rbp;
		sub rdi, rbp;
		sub rdi, rbp;
		add rdi, rdx; // pitch

		dec rcx;
		jmp nexth;
endh:
		//pop rbp;
		//pop rbx;
		//pop rdi;
		//pop rsi;
		emms;
	}
#else
	for (int h=0; h<src_height; h++)
	{
		for (int w=0; w<row_size/2; w+=2)
		{
			int w2 = w*2;
			int w3 = w*3;

			int y1 = psrcYUY2[w2];
			int u = psrcYUY2[w2+1];
			int y2 = psrcYUY2[w2+2];
			int v = psrcYUY2[w2+3];
			bufYUV24[w3]=y1;
			bufYUV24[w3+1]=u;
			bufYUV24[w3+2]=v;
			bufYUV24[w3+3]=y2;
			bufYUV24[w3+4]=u;
			bufYUV24[w3+5]=v;

		}
		psrcYUY2 += src_pitch;
		bufYUV24 += buf_pitch;
	}
#endif
}

void convertYUV24toYUY2(BYTE * pdestYUY2, int dest_pitch, int row_size, int dest_height,
			const BYTE * bufYUV24, int buf_pitch)
{ // convert interleaved not-subsampled YUV24 to YUY2
#if (ISSE)
	_asm {
		//push rsi;
		//push rdi;
		//push rbx;
		mov ecx, dest_height;
		mov eax, row_size; // double width
		shr rax, 1; // width in pixels
		mov ebx, dest_pitch;
		mov edx, buf_pitch;
		mov rdi, pdestYUY2;
		mov rsi, bufYUV24;
		pxor mm7,mm7;// 0
		//push rbp;
nexth:
		jecxz endh;

		xor rbp, rbp;
nextw:
		cmp rbp, rax;
		jge endw;
		movq mm0, [rsi]; // U2 Y3 V U Y2 V U Y1    high two bytes is not used
		punpcklbw mm0, mm7;
		pshufw mm0,mm0, 10110100b; //  V Y2 U Y1
		packuswb mm0, mm0;
		movd [rdi], mm0;
		add rsi, 6;
		add rdi, 4;
		add rbp, 2;
		jmp nextw;
endw:
		sub rsi, rbp;
		sub rsi, rbp;
		sub rsi, rbp;
		add rsi, rdx; // pitch

		sub rdi, rbp;
		sub rdi, rbp;
		add rdi, rbx; // pitch

		dec rcx;
		jmp nexth;
endh:
		//pop rbp;
		//pop rbx;
		//pop rdi;
		//pop rsi;
		emms;
	}
#else
	for (int h=0; h<dest_height; h++)
	{
		for (int w=0; w<row_size/2; w+=2)
		{
			int w2 = w*2;
			int w3 = w*3;

			int y1 = bufYUV24[w3];
			int u = bufYUV24[w3+1];
			int v = bufYUV24[w3+2];
			int y2 = bufYUV24[w3+3];
			pdestYUY2[w2] = y1;
			pdestYUY2[w2+1] = u;
			pdestYUY2[w2+2] = y2;
			pdestYUY2[w2+3] = v;

		}
		pdestYUY2 += dest_pitch;
		bufYUV24 += buf_pitch;
	}
#endif
}


//-------------------------------------------------------------------------------------------

PVideoFrame __stdcall ExInpaint::GetFrame(int n, IScriptEnvironment* env) {

    PVideoFrame maskframe;
	if (maskclip)
        maskframe = maskclip->GetFrame(n, env); // mask frame must be first to avoid makewritable bug

	PVideoFrame src = child->GetFrame(n, env);// Request frame 'n' from the child (source) clip.

	env->MakeWritable(&src); // will get results inplace

	int steps = 0;

	if (vi.IsYV12())
	{
		// This code deals with YV12 colourspace where the Y, U and V information are
		// stored in completely separate memory areas

		steps = inp->process3planes(src->GetWritePtr(PLANAR_Y),  src->GetPitch(PLANAR_Y),
			src->GetWritePtr(PLANAR_U), src->GetPitch(PLANAR_U),
			src->GetWritePtr(PLANAR_V),
			maskframe->GetReadPtr(PLANAR_Y), maskframe->GetPitch(PLANAR_Y),
			maskframe->GetReadPtr(PLANAR_U), maskframe->GetPitch(PLANAR_U),
			maskframe->GetReadPtr(PLANAR_V),
			xsize, ysize, radius, color, dilate, maxsteps); // inpaint frame

	}
	else if (vi.IsRGB24() || (vi.IsRGB32() && maskclip!=0) )
	{

		steps = inp->process(src->GetWritePtr(),  src->GetPitch(),
			maskframe->GetReadPtr(), maskframe->GetPitch(),
			xsize, ysize, radius, color, dilate, maxsteps); // inpaint frame

	}
	else if (vi.IsRGB32() && maskclip==0)
	{

		steps = inp->process(src->GetWritePtr(),  src->GetPitch(),
			0, 0,
			xsize, ysize, radius, color, dilate, maxsteps); // inpaint frame

	}
	else if ( vi.IsYUY2()  )
	{
		convertYUY2toYUV24(src->GetReadPtr(), src->GetPitch(), src->GetRowSize(), src->GetHeight(),
			bufferYUV, buffer_pitch);

		convertYUY2toYUV24(maskframe->GetReadPtr(), maskframe->GetPitch(), maskframe->GetRowSize(), maskframe->GetHeight(),
			buffermaskYUV, buffer_pitch);

		steps = inp->process(bufferYUV,  buffer_pitch,
			buffermaskYUV, buffer_pitch,
			xsize, ysize, radius, color, dilate, maxsteps); // inpaint frame

		convertYUV24toYUY2(src->GetWritePtr(), src->GetPitch(), src->GetRowSize(), src->GetHeight(),
			bufferYUV, buffer_pitch);
	}

	char buf[80];
	wsprintf(buf,"ExInpaint: frame=%d, steps=%d", n, steps);
	OutputDebugString(buf);

  // As we now are finished processing the image, we return the destination image.
	return src;
}

//-------------------------------------------------------------------------------------------

// This is the function that created the filter, when the filter has been called.
// This can be used for simple parameter checking, so it is possible to create different filters,
// based on the arguments recieved.

AVSValue __cdecl Create_ExInpaint(AVSValue args, void* user_data, IScriptEnvironment* env) {
    return new ExInpaint(args[0].AsClip(), // the 0th parameter is the source clip
		args[1].Defined() ? args[1].AsClip() : 0, // parameter mask
		 args[2].AsInt(0xFFFFFF), // parameter color of mask
		 args[3].AsInt(0), // parameter dilate
		 args[4].AsInt(8), // parameter xsize
		 args[5].AsInt(8), // parameter ysize
		 args[6].AsInt(0), // parameter search radius
		 args[7].AsInt(100000), // parameter max steps
		 env);
    // Calls the constructor with the arguments provied.
}

//-------------------------------------------------------------------------------------------

// The following function is the function that actually registers the filter in AviSynth
// It is called automatically, when the plugin is loaded to see which functions this filter contains.

const AVS_Linkage *AVS_linkage;

extern "C" __declspec(dllexport)
const char * __stdcall AvisynthPluginInit3(IScriptEnvironment *env, const AVS_Linkage *const vectors)
{
	AVS_linkage = vectors;
    env->AddFunction("ExInpaint", "c[mask]c[color]i[dilate]i[xsize]i[ysize]i[radius]i[steps]i", Create_ExInpaint, 0);
    // The AddFunction has the following parameters:
    // AddFunction(Filtername , Arguments, Function to call,0);

    // Arguments is a string that defines the types and optional names of the arguments for you filter.
    // c - Video Clip
    // i - Integer number
    // f - Float number
    // s - String
    // b - boolean


    return "ExInpaint plugin";

}

