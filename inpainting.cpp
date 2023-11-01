/* Exemplar-Based Inpainting

(c) 2008 Alexander Balakhnin (Fizick) http://avisynth.org.ru

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

   Algorithm is based on article:
Object Removal by Exemplar-Based Inpainting.
A. Criminisi, P. Perez, K. Toyama.
In Proc. Conf. Comp. Vision Pattern Rec., Madison, WI, Jun 2003.
http://research.microsoft.com/vision/cambridge/papers/Criminisi_cvpr03.pdf

   Source code is based on:
http://www.cc.gatech.edu/grads/q/qszhang/project/inpainting.htm */
/*Author: Qiushuang Zhang */
/*E-mail: qszhang@cc.gatech.edu */
/*Nov.29, 2005 */

/* Modifications (changes):

v0.1 at 10-12 January 2008 by Fizick:
 - replaced double to integer (scaled) for speed
 - removed image library
 - adopted to AviSynth API, RRB32, YV12
 - fixed wrong gray, small bugs
 - different winx, winy sizes
 - licensed under GNU GPL

v0.2 at 13-26 January 2008 by Fizick:
 - even patch sizes instead of odd (with some approriate changes)
 - assembler MMX optimization of PatchTexture
 - use SAD instead of SSD
 - local max priority updating
 - option to estimate search radius automatically
 - added YUY2 format and YUV24
 - added dilation of mask

*/

#include "inpainting.h"
#include <memory.h>
#include <math.h>

#include <windows.h> // for wsprintf and OutpuDebugString only

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

inpainting::inpainting(int _width, int _height, int _pixel_format)
{
	m_width = _width;
	m_height = _height;
	pixel_format = _pixel_format;

	m_mark = new unsigned char[m_width*m_height];
	m_confid = new int[m_width*m_height];
	m_pri = new int[m_width*m_height];
	m_source = new unsigned char[m_width*m_height];

	if(pixel_format == RGB32 || pixel_format == RGB24 || pixel_format == RGBA || pixel_format == YUY2 || pixel_format == YUV24)
		m_gray  = new unsigned char[m_width*m_height];

}


inpainting::~inpainting(void)
{
	if(m_mark)delete [] m_mark;
	if(m_confid)delete [] m_confid;
	if(m_pri)delete [] m_pri;
	if(m_source)delete [] m_source;
	if(m_gray && pixel_format != YV12 )delete [] m_gray;
}



/*********************************************************************/
int inpainting::process(unsigned char * _psrc, int _src_pitch,
						const unsigned char * _maskp, int _mask_pitch,
					   int _xsize, int _ysize, int _radius, int _maskcolor, int _dilateflags, int maxsteps)
{ // wrapper for interleave

	return process3planes(_psrc, _src_pitch,
					   0, 0,
					   0,
						_maskp, _mask_pitch,
						0, 0,
						0,
					    _xsize, _ysize, _radius, _maskcolor, _dilateflags, maxsteps);

}
/*********************************************************************/
int inpainting::process3planes(unsigned char * _psrc, int _src_pitch,
					   unsigned char * _psrcU, int _src_pitchU,
					   unsigned char * _psrcV,
						const unsigned char * _maskp, int _mask_pitch,
						const unsigned char * _maskpU, int _mask_pitchU,
						const unsigned char * _maskpV,
					   int _xsize, int _ysize, int _radius, int _maskcolor, int _dilateflags, int maxsteps)
{// the main function to process the whole image

	psrc = _psrc;
	src_pitch = _src_pitch;
	psrcU = _psrcU;
	src_pitchUV = _src_pitchU;
	// assume pitchV=pitchU (it is always in AviSynth) for speed
	psrcV = _psrcV;
	pmask = _maskp;
	mask_pitch = _mask_pitch;
	pmaskU = _maskpU;
	mask_pitchUV = _mask_pitchU;
	pmaskV = _maskpV;
	winxsize = _xsize/2; // window is half of full side size
	winysize = _ysize/2;
	radius = _radius; // 0 for auto search, radius > size
	maskcolor = _maskcolor;
	dilateflags = _dilateflags;

	m_top = m_height;  // initialize the rectangle area
    m_bottom = 0;
	m_left = m_width;
	m_right = 0;

	Convert2Gray();  // create  gray image from RGB source
	memset( m_confid, 0, m_width*m_height*sizeof(int) ); // init
	GetMask();
	if (dilateflags)
        Dilate(dilateflags);
	//char buf[80];
	if (radius==0)
	{
		radius = EstimateRadius(); // first approximation
	//wsprintf(buf,"ExInpaint: radius=%d", radius);
	//OutputDebugString(buf);
		radius = MAX((radius + 5), ((MIN(winxsize, winysize)) * 4)); // semi-empirical min estimation
	//wsprintf(buf,"ExInpaint: radius=%d", radius);
	//OutputDebugString(buf);
	}
	DrawBoundary();  // first time draw boundary
	draw_source();   // find the patches that can be used as sample texture
	memset(m_pri, 0, m_width*m_height*sizeof(int));
	for(int j= m_top; j<=m_bottom; j++)
	    for(int i = m_left; i<= m_right; i++)
			if(m_mark[j*m_width+i] == BOUNDARY)
				m_pri[j*m_width+i] = priority(i,j);//if it is boundary, calculate the priority
	int count=0;
	max_pri = -1; // init as not ready
	while(TargetExist() && count<maxsteps)
	{
		count++;
		if (max_pri<0) // if not ready from prev step local updating, do full search
			max_pri = HighestPriority(); // get new pri_x. pri_y
//	char buf[80];
//	wsprintf(buf,"Inpaint: pri_x=%d, pri_y=%d, max_pri=%d", pri_x, pri_y, max_pri);
//	OutputDebugString(buf);
		if (max_pri<0) // if not valid, then
			return -count; // probably bad mask, no boundary (e.g. full frame is mask), return
		int patch_x, patch_y;
		bool found = PatchTexture(pri_x, pri_y, patch_x, patch_y);  // find the most similar source patch
		if (!found)
			return count; // patch not found at this step
		int conf = ComputeConfidence(pri_x,pri_y); // update confidence
		update(pri_x, pri_y, patch_x,patch_y, conf );// inpaint this area
		UpdateBoundary(pri_x, pri_y); // update boundary near the changed area
		max_pri = UpdatePri(pri_x, pri_y);  //  update priority near the changed area
		// if new max>0 then it is ready in patched area as well as new pri_x and pri_y
	}
	return count; // number of inpainting steps (iterations)
}


/*********************************************************************/
void inpainting::Convert2Gray(void)
{
	unsigned char *psrc1 = psrc;

	if(pixel_format == RGB32 || pixel_format == RGBA)
	{
		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				int b = psrc1[x*4];
				int g = psrc1[x*4+1];
				int r = psrc1[x*4+2];
				m_gray[y*m_width+x] = ((b*3735 + g*19268 + r*9765)/32768);
			}
			psrc1 += src_pitch;
		}
	}
	else if (pixel_format == RGB24)
	{
		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				int b = psrc1[x*3];
				int g = psrc1[x*3+1];
				int r = psrc1[x*3+2];
				m_gray[y*m_width+x] = ((b*3735 + g*19268 + r*9765)/32768);
			}
			psrc1 += src_pitch;
		}
	}
	else if (pixel_format == YV12)
		m_gray = psrc; // gray is simply pointer to luma
	else if (pixel_format == YUY2)
	{
		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				m_gray[y*m_width+x] = psrc1[x<<1];
			}
			psrc1 += src_pitch;
		}
	}
	else if (pixel_format == YUV24)
	{
		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				m_gray[y*m_width+x] = psrc1[x+x+x];
			}
			psrc1 += src_pitch;
		}
	}
}

/*********************************************************************/
int inpainting::EstimateRadius(void)// estimate redius by iterative erosion (Fizick)
{


	// assume mark data is SOURCE or TARGET
	// temporary set ERODED pixels

	int iter = 0;
	bool targets_exist;

	do // erode loops
	{
		iter++;
		targets_exist = false;

		int j= 0;// top
		{
			int i=0;// leftmost
			if(m_mark[j*m_width+i]==TARGET)
			{
				if (m_mark[j*m_width+i+1]==SOURCE || m_mark[(j+1)*m_width+i]==SOURCE ||
					m_mark[j*m_width+i+1]==ERODED || m_mark[(j+1)*m_width+i]==ERODED)
					m_mark[j*m_width+i] = ERODEDNEXT;
				targets_exist=true;
			}
			for(i = 1; i< m_width-1; i++) // middle i
			{
				if(m_mark[j*m_width+i]==TARGET)
				{
					//if one of the four neighbours is not target (i.e. source or eroded) pixel, then this should be eroded
					if(	m_mark[j*m_width+i-1]==SOURCE ||
						m_mark[j*m_width+i+1]==SOURCE || m_mark[(j+1)*m_width+i]==SOURCE ||
						m_mark[j*m_width+i-1]==ERODED ||
						m_mark[j*m_width+i+1]==ERODED || m_mark[(j+1)*m_width+i]==ERODED)
							m_mark[j*m_width+i] = ERODEDNEXT;
					targets_exist=true;
				}
			}
			i= m_width-1; // rightmost
			if(m_mark[j*m_width+i]==TARGET)
			{
				if(	m_mark[j*m_width+i-1]==SOURCE || m_mark[(j+1)*m_width+i]==SOURCE ||
					m_mark[j*m_width+i-1]==ERODED || m_mark[(j+1)*m_width+i]==ERODED)
					m_mark[j*m_width+i] = ERODEDNEXT;
				targets_exist=true;
			}
		}

		// middle j
		for(j= 1; j< m_height-1; j++)
		{
			int i=0;// leftmost
			if(m_mark[j*m_width+i]==TARGET)
			{
				if(	m_mark[(j-1)*m_width+i]==SOURCE	||
					m_mark[j*m_width+i+1]==SOURCE	|| m_mark[(j+1)*m_width+i]==SOURCE ||
					m_mark[(j-1)*m_width+i]==ERODED	||
					m_mark[j*m_width+i+1]==ERODED	|| m_mark[(j+1)*m_width+i]==ERODED)
						m_mark[j*m_width+i] = ERODEDNEXT;
				targets_exist=true;
			}
			for(i = 1; i< m_width-1; i++) // middle
			{
				if(m_mark[j*m_width+i]==TARGET)
				{
					//if one of the four neighbours is not target (i.e. source or eroded) pixel, then this should be eroded
					if(	m_mark[(j-1)*m_width+i]==SOURCE	|| m_mark[j*m_width+i-1]==SOURCE ||
						m_mark[j*m_width+i+1]==SOURCE	|| m_mark[(j+1)*m_width+i]==SOURCE ||
						m_mark[(j-1)*m_width+i]==ERODED	|| m_mark[j*m_width+i-1]==ERODED ||
						m_mark[j*m_width+i+1]==ERODED	|| m_mark[(j+1)*m_width+i]==ERODED)
							m_mark[j*m_width+i] = ERODEDNEXT;
					targets_exist=true;
				}
			}
			i= m_width-1; // rightmost
			if(m_mark[j*m_width+i]==TARGET)
			{
				if(	m_mark[(j-1)*m_width+i]==SOURCE	|| m_mark[j*m_width+i-1]==SOURCE ||
					m_mark[(j+1)*m_width+i]==SOURCE ||
					m_mark[(j-1)*m_width+i]==ERODED	|| m_mark[j*m_width+i-1]==ERODED ||
					m_mark[(j+1)*m_width+i]==ERODED)
						m_mark[j*m_width+i] = ERODEDNEXT;
				targets_exist=true;
			}
			// replace ERODENNEXT to ERODED in prev line for next iteration
			for(i = 0; i< m_width; i++)
				if(m_mark[(j-1)*m_width+i]==ERODEDNEXT)	m_mark[(j-1)*m_width+i]=ERODED;
		}


		j= m_height-1;// bottom
		{
			int i=0;
			if(m_mark[j*m_width+i]==TARGET)
			{
				if(	m_mark[(j-1)*m_width+i]==SOURCE	|| m_mark[j*m_width+i+1]==SOURCE ||
					m_mark[(j-1)*m_width+i]==ERODED	|| m_mark[j*m_width+i+1]==ERODED)
						m_mark[j*m_width+i] = ERODEDNEXT;
				targets_exist=true;
			}
			for(i = 1; i< m_width-1; i++)// middle
			{
				if(m_mark[j*m_width+i]==TARGET)
				{
					//if one of the four neighbours is not target (i.e. source or eroded) pixel, then this should be eroded
					if(	m_mark[(j-1)*m_width+i]==SOURCE	|| m_mark[j*m_width+i-1]==SOURCE ||
						m_mark[j*m_width+i+1]==SOURCE ||
						m_mark[(j-1)*m_width+i]==ERODED	|| m_mark[j*m_width+i-1]==ERODED ||
						m_mark[j*m_width+i+1]==ERODED)
							m_mark[j*m_width+i] = ERODEDNEXT;
					targets_exist=true;
				}
			}
			i= m_width-1; // rightmost
			if(m_mark[j*m_width+i]==TARGET)
			{
				if(	m_mark[(j-1)*m_width+i]==SOURCE	|| m_mark[j*m_width+i-1]==SOURCE ||
					m_mark[(j-1)*m_width+i]==ERODED	|| m_mark[j*m_width+i-1]==ERODED)
						m_mark[j*m_width+i] = ERODEDNEXT;
				targets_exist=true;
			}
			// replace ERODENNEXT to ERODED in prev line for next iteration
			for(i = 0; i< m_width; i++)
				if(m_mark[(j-1)*m_width+i]==ERODEDNEXT)	m_mark[(j-1)*m_width+i]=ERODED;
			// replace ERODENNEXT to ERODED in last line for next iteration
			for(i = 0; i< m_width; i++)
				if(m_mark[j*m_width+i]==ERODEDNEXT)	m_mark[j*m_width+i]=ERODED;
		}
	} while (targets_exist);


		// restore all ERODED to TARGET
		for(int j= 0; j< m_height; j++)
		{
			for(int i = 0; i< m_width; i++) // middle
			{
				if(m_mark[j*m_width+i]==ERODED)
					m_mark[j*m_width+i]=TARGET;

			}
		}

	return iter; // erode count as a radius
}
/*********************************************************************/
void inpainting::Dilate(int dilateflags)// dilate the mask by 1 pixel
{
    // here I use ERODED as mark for dilated :-) to not add extra constant
   
	unsigned char * pmark = m_mark;
    
	if (dilateflags & 1 ) // horizontal dilate
    {
		int j;
        for(j= 0; j< m_height; j++)
        {
            int i = 0;
            if(pmark[i]==SOURCE && pmark[i+1]==TARGET)
                pmark[i] = ERODED;

            for(i = 1; i< m_width-1; i++)
            {
                if(pmark[i]==SOURCE && (pmark[i+1]==TARGET || pmark[i-1]==TARGET) )
                     pmark[i] = ERODED;
            }
            i = m_width-1; // last
            if(pmark[i]==SOURCE && pmark[i-1]==TARGET)
                pmark[i] = ERODED;

            pmark += m_width;
        }
    }
    
	if (dilateflags & 2) // vertical dilate
    {
        pmark = m_mark; // re-set pointer
        int j=0;
        int i;
        for(i = 0; i< m_width; i++)
        {
            if(pmark[i]==SOURCE && pmark[m_width+i]==TARGET)
                 pmark[i] = ERODED;
        }
        pmark += m_width;
        for(j= 1; j< m_height-1; j++)
        {
            for(i = 0; i< m_width; i++)
            {
                if(pmark[i]==SOURCE && (pmark[m_width+i]==TARGET || pmark[-m_width+i]==TARGET) )
                    pmark[i] = ERODED;
            }
            pmark += m_width;
        }
        for(i = 0; i< m_width; i++)
        {
            if(pmark[i]==SOURCE && pmark[-m_width+i]==TARGET)
                pmark[i] = ERODED;
        }
    }

    if (dilateflags & 3 ) // if any dilate then set all ERODED (i.e.dilated) pixels to TARGET
	{
	    pmark = m_mark; // re-set pointer
		for(int j= 0; j< m_height; j++)
		{
			for(int i = 0; i< m_width; i++)
			{
				if(pmark[i]==ERODED)
					pmark[i]=TARGET;
			}
            pmark[j] += m_width;
		}
	}

}



/*********************************************************************/
int inpainting::HighestPriority()
{
	// find the boundary pixel with highest priority
	int max_pri1 = -1; // local,  m_pri may be 0 in flat regions (Fizick)

	unsigned char* pmark = m_mark + m_width*m_top + m_left; // pointers
	int * ppri = m_pri + m_width*m_top + m_left;

	int pri_x1 = 0; // local vars
	int pri_y1 = 0;

	int jm = m_bottom - m_top+1;
	int im = m_right - m_left+1;

	for(int j= 0; j<jm; j++)
	{
	    for(int i = 0; i< im; i++)
		{
			if(pmark[i] == BOUNDARY && ppri[i]>max_pri1)
			{
				max_pri1 = ppri[i];
				pri_x1 = i;
				pri_y1 = j;
			}
		}
		pmark += m_width;
		ppri += m_width;
	}

	pri_x = pri_x1 + m_left; // restore offset for global
	pri_y = pri_y1 + m_top;

	return max_pri1;
}

/*********************************************************************/
void inpainting::GetMask(void)// first time mask
{
	int confid1 = 2048;// scaled  for int division

	if (pixel_format == RGB32)
	{
		const unsigned int * intmask = reinterpret_cast<const unsigned int *>(pmask);
		int intmask_pitch = mask_pitch/4;

		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				// get mask data for this point
				unsigned int color = *(intmask + x) & 0xFFFFFF; // without alpha
				if((int)color == maskcolor)// if the pixel is specified as mask
				{
					m_mark[y*m_width+x] = TARGET;
					m_confid[y*m_width+x] = 0;
				}
				else {
					m_mark[y*m_width+x] = SOURCE;
					m_confid[y*m_width+x] = confid1;
				}
			}
			intmask += intmask_pitch;
		}
	}
	else if (pixel_format == RGBA) // really ARGB in avisynth (Alpha - high byte)
	{
		const unsigned char * pmask1 = psrc; // use source clip, not mask

		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				// get mask data for this point
				int color = *(pmask1 + x*4 + 3) ; // alpha
				if(color > 127)// if the pixel is specified as mask, unike other modes, use threshold
				{
					m_mark[y*m_width+x] = TARGET;
					m_confid[y*m_width+x] = 0;
				}
				else {
					m_mark[y*m_width+x] = SOURCE;
					m_confid[y*m_width+x] = confid1;
				}
			}
			pmask1 += src_pitch;
		}
	}
	else if (pixel_format == RGB24)
	{
		const unsigned char * pmask1 = pmask;

		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				// get mask data for this point
				int color = *(pmask1 + x*3) | *(pmask1 + x*3 + 1)<<8 | *(pmask1 + x*3 + 2)<<16; // bgr
				if(color == maskcolor)// if the pixel is specified as mask
				{
					m_mark[y*m_width+x] = TARGET;
					m_confid[y*m_width+x] = 0;
				}
				else {
					m_mark[y*m_width+x] = SOURCE;
					m_confid[y*m_width+x] = confid1;
				}
			}
			pmask1 += mask_pitch;
		}
	}
	else if (pixel_format == YV12)
	{
		const unsigned char * pmask1 = pmask;
		const unsigned char * pmaskU1 = pmaskU;
		const unsigned char * pmaskV1 = pmaskV;

		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				// get mask data for this point
				int color = pmaskV1[x>>1] | pmaskU1[x>>1]<<8 | pmask1[x]<<16; // yuv
				if(color == maskcolor)// if the pixel is specified as mask
				{
					m_mark[y*m_width+x] = TARGET;
					m_confid[y*m_width+x] = 0;
				}
				else {
					m_mark[y*m_width+x] = SOURCE;
					m_confid[y*m_width+x] = confid1;
				}
			}
			pmask1 += mask_pitch;
			if(y%2) // chroma is vertically subsampled too
			{
				pmaskU1 += mask_pitchUV;
				pmaskV1 += mask_pitchUV;
			}
		}
	}
	else if (pixel_format == YUY2)
	{
		const unsigned char * pmask1 = pmask;

		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x+=2) // 2 pixel
			{
				// get mask data for this point
				int U = pmask1[(x<<1)+1];
				int V = pmask1[(x<<1)+3];
				int color = V | U<<8 | pmask1[x<<1]<<16; // yuv
				if(color == maskcolor)// if the pixel is specified as mask
				{
					m_mark[y*m_width+x] = TARGET;
					m_confid[y*m_width+x] = 0;
				}
				else {
					m_mark[y*m_width+x] = SOURCE;
					m_confid[y*m_width+x] = confid1;
				}
				color = V | U<<8 | pmask1[(x<<1)+2]<<16; // second yuv
				if(color == maskcolor)// if the pixel is specified as mask
				{
					m_mark[y*m_width+x+1] = TARGET;
					m_confid[y*m_width+x+1] = 0;
				}
				else {
					m_mark[y*m_width+x+1] = SOURCE;
					m_confid[y*m_width+x+1] = confid1;
				}
			}
			pmask1 += mask_pitch;
		}
	}
	else if (pixel_format == YUV24)
	{
		const unsigned char * pmask1 = pmask;

		for(int y = 0; y<m_height; y++)
		{
			for(int x = 0; x<m_width; x++)
			{
				// get mask data for this point
				int color = *(pmask1 + 2 + x*3) | *(pmask1 + x*3 + 1)<<8 | *(pmask1 + x*3 + 0)<<16; // vuy
				if(color == maskcolor)// if the pixel is specified as mask
				{
					m_mark[y*m_width+x] = TARGET;
					m_confid[y*m_width+x] = 0;
				}
				else {
					m_mark[y*m_width+x] = SOURCE;
					m_confid[y*m_width+x] = confid1;
				}
			}
			pmask1 += mask_pitch;
		}
	}

}
/*********************************************************************/
void inpainting::DrawBoundary(void)// fist time draw boundary
{

	for(int j= 0; j< m_height; j++)
	    for(int i = 0; i< m_width; i++)
		{
			if(m_mark[j*m_width+i]==TARGET)
			{
				if(i<m_left)m_left = i; // rrsize the rectangle to the range of target area
				if(i>m_right)m_right = i;
				if(j>m_bottom)m_bottom = j;
				if(j<m_top)m_top = j;
				//if one of the four neighbours is source pixel, then this should be a boundary
				if(j==m_height-1||j==0||i==0||i==m_width-1||m_mark[(j-1)*m_width+i]==SOURCE||m_mark[j*m_width+i-1]==SOURCE
					||m_mark[j*m_width+i+1]==SOURCE||m_mark[(j+1)*m_width+i]==SOURCE)m_mark[j*m_width+i] = BOUNDARY;
			}
		}
}



/*********************************************************************/
int inpainting::priority(int i, int j)
{
	int confidence, data;
	confidence = ComputeConfidence(i,j); // confidence term
	data = ComputeData(i,j);// data term
	return confidence*data;
}

/*********************************************************************/
int inpainting::ComputeConfidence(int i, int j)
{
	int confidence=0;
	for(int y = MAX(j -winysize,0); y< MIN(j+winysize,m_height); y++)
		for(int x = MAX(i-winxsize,0); x<MIN(i+winxsize, m_width); x++)
			confidence+= m_confid[y*m_width+x];
	confidence /= (winxsize*2)*(winysize*2);
	return confidence;

}
/*********************************************************************/
int inpainting::ComputeData(int i, int j)
{
	gradient grad, temp, grad_T;
	grad.grad_x=0;
	grad.grad_y=0;
	int result;
	int magnitude;
	int magmax=0;
	int x, y;
	for(y = MAX(j -winysize,0); y< MIN(j+winysize,m_height); y++)
	{
		for( x = MAX(i-winxsize,0); x<MIN(i+winxsize, m_width); x++)
		{
			// find the greatest gradient in this patch, this will be the gradient of this pixel(according to "detail paper")
			if(m_mark[y*m_width+x] == SOURCE) // source pixel
			{
				//since I use four neighbors to calculate the gradient, make sure this four neighbors do not touch target region(big jump in gradient)
				if( (x+1<m_width && m_mark[y*m_width+x+1]!=SOURCE) // add bound check (Fizick)
					|| (x-1>=0 && m_mark[y*m_width+x-1]!=SOURCE)
					|| (y+1<m_height && m_mark[(y+1)*m_width+x]!=SOURCE)
					|| (y-1>=0 && m_mark[(y-1)*m_width+x]!=SOURCE))
					continue;
 				temp = GetGradient(x,y);
				magnitude = temp.grad_x*temp.grad_x+temp.grad_y*temp.grad_y;
				if(magnitude>magmax)
				{
					grad.grad_x = temp.grad_x;
					grad.grad_y = temp.grad_y;
					magmax = magnitude;
				}
			}
		}
	}
		grad_T.grad_x = grad.grad_y;// perpendicular to the gradient: (x,y)->(y, -x)
		grad_T.grad_y = -grad.grad_x;

	norm nn = GetNorm(i,j);
	result = nn.norm_x*grad_T.grad_x+nn.norm_y*grad_T.grad_y; // dot product
//	result/=255; //"alpha" in the paper: normalization factor (it is not important, disable for interger)
	result = abs(result);
	return result;
}


/*********************************************************************/

gradient inpainting::GetGradient(int i, int j)
{
    // scale gradient to factor 2 to be integer in v.0.1, add bound check (Fizick)

    // sum by 2 point in v0.2 (for even window without center point). It is also more stable to noise.

	gradient result;

	if (i==0 && j==0)
	{
	    result.grad_x = ((int)m_gray[1] - (int)m_gray[0])*2;
        result.grad_y = ((int)m_gray[m_width] - (int)m_gray[0])*2;
	}
	else if (i==0)
	{
	    result.grad_x = ((int)m_gray[j*m_width+1] - (int)m_gray[j*m_width])
            + ((int)m_gray[(j-1)*m_width+1] - (int)m_gray[(j-1)*m_width]);
        result.grad_y = ((int)m_gray[(j)*m_width] - (int)m_gray[(j-1)*m_width])*2;
	}
	else if (j==0)
	{
	    result.grad_x = ((int)m_gray[i] - (int)m_gray[i-1])*2;
        result.grad_y = ((int)m_gray[m_width +i] - (int)m_gray[i])
            + ((int)m_gray[m_width +i-1] - (int)m_gray[i-1]);
	}
	else
	{
	    result.grad_x = ((int)m_gray[j*m_width+i] - (int)m_gray[j*m_width+i-1])
            + ((int)m_gray[(j-1)*m_width+i] - (int)m_gray[(j-1)*m_width+i-1]);
        result.grad_y = ((int)m_gray[(j)*m_width +i] - (int)m_gray[(j-1)*m_width+i])
            + ((int)m_gray[(j)*m_width +i-1] - (int)m_gray[(j-1)*m_width+i-1]);
	}

	return result;
}

/*********************************************************************/
norm inpainting::GetNorm(int i, int j)
{
	norm result;
	int num=0;
	int neighbor_x[9];
	int neighbor_y[9];
	int record[9];
	int count = 0;
	for(int y = MAX(j-1,0); y<MIN(j+1,m_height); y++)
	{
		for(int x = MAX(i-1,0); x<MIN(i+1,m_width); x++)
		{
			count++;
			if(x==i&&y==j)continue;
			if(m_mark[y*m_width+x]==BOUNDARY)
			{
				num++;
				neighbor_x[num] = x;
				neighbor_y[num] = y;
				record[num]=count;
			}
		}
	}
		if(num==0||num==1) // if it doesn't have two neighbors, give it a random number to proceed
		{
			result.norm_x = 181;//0.6;
			result.norm_y = 182;//0.8;
			return result;
		}
		// draw a line between the two neighbors of the boundary pixel, then the norm is the perpendicular to the line
			int n_x = neighbor_x[2]-neighbor_x[1];
			int n_y = neighbor_y[2]-neighbor_y[1];
			int temp=n_x;
			n_x = n_y;
			n_y = temp;
			double square = sqrt(double(n_x*n_x + n_y*n_y));

	result.norm_x = n_x*256/square; // scaled int to divide
	result.norm_y = n_y*256/square;
	return result;
}

/*********************************************************************/
bool inpainting::draw_source(void)
{
	// draw a window around the pixel, if all of the points within the window are source pixels, then this patch can be used as a source patch
	bool flag;
	for(int j = 0; j<m_height; j++)
	{
		for(int i = 0; i<m_width; i++)
		{
			flag=true;
			if(i<winxsize||j<winysize||i>m_width-winxsize||j>m_height-winysize)
				m_source[j*m_width+i]=0;//cannot form a complete window
			else
			{
				for(int y = j-winysize; y<j+winysize; y++)
				{
					for(int x = i-winxsize; x<i+winxsize; x++)
					{
						if(m_mark[y*m_width+x]!=SOURCE)
						{
							m_source[j*m_width+i]=0;
							flag = false;
							break;
						}
					}
					if(flag==false)break;
				}
			    if(flag!=false)m_source[j*m_width+i]=1;
			}
		}
	}
	return true;
}

/*********************************************************************/
bool inpainting::PatchTexture(int x, int y, int &patch_x, int &patch_y)
{
	// find the most similar patch, according to SSD

#define MIN_INITIAL 99999999

    int ymin, ymax, xmin, xmax;

	if (radius>0) // added by Fizick
    {
        ymin = MAX(y-radius, 0);
        ymax = MIN(y+radius, m_height);
        xmin = MAX(x-radius, 0);
        xmax = MIN(x+radius, m_width);
    }
    else // full frame search (slow)
    {
        ymin = 0;
        ymax = m_height;
        xmin = 0;
        xmax = m_width;
    }


	unsigned char *psrc1 = psrc;
	int winxsize1 = winxsize;

	long min=MIN_INITIAL;
	long sum;
	int source_x, source_y;
	int target_x, target_y;

	if(pixel_format == RGB32 || pixel_format == RGBA)
	{

		for(int j = ymin; j<ymax; j++)
		{
			for(int i = xmin; i<xmax; i++)
			{
				if(m_source[j*m_width+i]==0)continue; // not good patch source
				sum=0;

				for(int iter_y=MAX(-winysize, -y); iter_y<MIN(winysize, m_height-y); iter_y++)
				{
					source_y = j+iter_y;
					target_y = y+iter_y;

					unsigned char * tysrc = psrc + target_y*src_pitch;
					unsigned char * sysrc = psrc + source_y*src_pitch;
					unsigned char * tymark = m_mark + target_y*m_width;

					if (x-winxsize<0 || x+winxsize>m_width) // process border separately to process middle without checking (faster)
					{

						for(int iter_x=(-1)*winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							if(target_x<0||target_x>=m_width)continue;

							// it is the most time-comsuming part of code:
							if(tymark[target_x]==SOURCE) // compare
							{
								int temp_b = tysrc[target_x*4]-sysrc[source_x*4];
								int temp_g = tysrc[target_x*4+1]-sysrc[source_x*4+1];
								int temp_r = tysrc[target_x*4+2]-sysrc[source_x*4+2];

//								sum += temp_r*temp_r + temp_g*temp_g + temp_b*temp_b; // SSD
								sum += (abs(temp_r) + abs(temp_g) + abs(temp_b)) ; // SAD
							}
						}
					}
					else // middle
					{
#if (!ISSE)
						for(int iter_x=-winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							// it is the most time-comsuming part of code:
							int smark = -(int)(tymark[target_x]==SOURCE); // compare, remove jump for speed
							{
								int temp_b = (int)tysrc[target_x*4]-sysrc[source_x*4];
								int temp_g = (int)tysrc[target_x*4+1]-sysrc[source_x*4+1];
								int temp_r = (int)tysrc[target_x*4+2]-sysrc[source_x*4+2];

//								sum += (temp_r*temp_r + temp_g*temp_g + temp_b*temp_b) & smark; // SSD
								sum += (abs(temp_r) + abs(temp_g) + abs(temp_b)) & smark; // SAD
							}
						}
#else
						_asm
						{
							//push rsi;
							//push rdi;
							//push rbx;
							// assume winxsize is multiple 2
							mov rsi, sysrc;
							mov rdi, tysrc;
							mov eax, i;
							mov ecx, winxsize1;
							sub rax, rcx; // source_x = i + iter_x (start iter_x = -winxsize)
							mov edx, x;
							sub rdx, rcx; // target_x = x + iter_x
							mov rbx, tymark;
							pxor mm0, mm0; // constant 0
							pcmpeqd mm1, mm1; // constant two FFFFFFFF
							pxor mm2, mm2; // 0, will be our block sum as low word
							psrld mm1, 8; // constant two 00FFFFFF with 00 at alpha
align 16
startRGBA:
							movd mm3, [rbx + rdx]; // read 4 pixel mark, but will use 2 only
							punpcklbw mm3, mm0; // bytes to 4 words
							punpcklwd mm3, mm0; // words to 2 doublewords
							movq mm4, [rsi + rax*4]; // read 2 source pixels by 4 bytes BGRA
							pcmpeqd mm3, mm0;// compare every doubleword mark with 0
							movq mm5, [rdi + rdx*4]; // read 2 target pixels by 4 bytes BGRA
							pand mm3, mm1; // final mask of mark and alpha
							pand mm4, mm3;// clear masked bytes
							pand mm5, mm3;// clear masked bytes
							psadbw mm4, mm5; // get  summ of absolute differenses SAD of 2 masked RGB pixels
							paddw mm2, mm4; // add to block sum
							add rdx, 2; // nex mark
							add rax, 2; // next 2 pixels
							dec rcx;
							jrcxz endRGBA;
							jmp startRGBA;
endRGBA:
							movd mm6, sum;// get old sum of block
							punpcklwd mm2, mm0; // words with partial sum to doublewords, use low
							paddd mm6, mm2; // add to block sum
							movd sum, mm6; // update sum of block
							//pop rbx;
							//pop rdi;
							//pop rsi;
						}
#endif
					}
				}
				if(sum<min)
				{
					min=sum;
					patch_x = i;
					patch_y = j;
				}
			}
		}
	}
	else if(pixel_format == RGB24 || pixel_format == YUV24 )
	{
		for(int j = ymin; j<ymax; j++)
		{
			for(int i = xmin; i<xmax; i++)
			{
				if(m_source[j*m_width+i]==0)continue; // not good patch source
				sum=0;
				for(int iter_y=MAX(-winysize, -y); iter_y<MIN(winysize, m_height-y); iter_y++)
				{
					source_y = j+iter_y;
					target_y = y+iter_y;

					unsigned char * tysrc = psrc + target_y*src_pitch;
					unsigned char * sysrc = psrc + source_y*src_pitch;
					unsigned char * tymark = m_mark + target_y*m_width;

					if (x-winxsize<0 || x+winxsize>m_width) // process border separately to process middle without checking (faster)
					{

						for(int iter_x=(-1)*winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							if(target_x<0||target_x>=m_width)continue;

							// it is the most time-comsuming part of code:
							if(tymark[target_x]==SOURCE) // compare
							{
								int temp_b = tysrc[target_x*3]-sysrc[source_x*3];
								int temp_g = tysrc[target_x*3+1]-sysrc[source_x*3+1];
								int temp_r = tysrc[target_x*3+2]-sysrc[source_x*3+2];

//								sum += temp_r*temp_r + temp_g*temp_g + temp_b*temp_b; // SSD
								sum += (abs(temp_r) + abs(temp_g) + abs(temp_b)) ; // SAD
							}
						}
					}
					else // middle
					{
#if (!ISSE)
						for(int iter_x=-winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							// it is the most time-comsuming part of code:
							int smark = -(int)(tymark[target_x]==SOURCE); // compare, remove jump for speed
							{
								int temp_b = (int)tysrc[target_x*3]-sysrc[source_x*3];
								int temp_g = (int)tysrc[target_x*3+1]-sysrc[source_x*3+1];
								int temp_r = (int)tysrc[target_x*3+2]-sysrc[source_x*3+2];

//								sum += (temp_r*temp_r + temp_g*temp_g + temp_b*temp_b) & smark; // SSD
								sum += (abs(temp_r) + abs(temp_g) + abs(temp_b)) & smark; // SAD
							}
						}
#else
						_asm
						{
							//push rsi;
							//push rdi;
							//push rbx;
							// assume winxsize is multiple 2
							mov rsi, sysrc;
							mov rdi, tysrc;
							mov eax, i;
							mov ecx, winxsize1;
							sub rax, rcx; // source_x = i + iter_x (start iter_x = -winxsize)
							mov edx, x;
							sub rdx, rcx; // target_x = x + iter_x
							mov rbx, tymark;
							pxor mm0, mm0; // constant 0
							pcmpeqd mm1, mm1; // constant FFFFFFFF FFFFFFFF
							pxor mm2, mm2; // 0, will be our block sum as low word
							psrlq mm1, 16; // constant two 0000FFFF FFFFFFFF with ..rgbrgb
							//push rbp; // save rbp
							mov rbp, rax;
							add rbp, rax;
							add rax, rbp; // rax = rax*3
							mov rbp, rdx;
							add rbp, rdx;
							add rbp, rdx; // rbp = rdx*3

align 16
startRGB24:
							movd mm3, [rbx + rdx]; // read 4 pixel mark, but will use 2 only
							punpcklbw mm3, mm0; // bytes to 4 words
							punpcklwd mm3, mm0; // low words to 2 doublewords
							pcmpeqd mm3, mm0;// compare every doubleword mark with 0
							psrlq mm3, 8; // shift mask to .2222111
							movq mm4, [rsi + rax]; // read 2 source pixels by 3 bytes BGR and some extra 2 bytes
							movq mm5, [rdi + rbp]; // read 2 target pixels by 3 bytes BGR and some extra 2 bytes
							pand mm3, mm1; // final mask of mark and pix
							pand mm4, mm3;// clear masked bytes
							pand mm5, mm3;// clear masked bytes
							psadbw mm4, mm5; // get  summ of absolute differenses SAD of 2 masked RGB pixels
							paddw mm2, mm4; // add to block sum
							add rdx, 2; // nex mark
							add rax, 6; // next 2 pixels
							add rbp, 6
							dec rcx;
							jrcxz endRGB24;
							jmp startRGB24;
endRGB24:
							//pop rbp; // restore rbp for sum parameter
							movd mm6, sum;// get old sum of block
							punpcklwd mm2, mm0; // words with partial sum to doublewords, use low
							paddd mm6, mm2; // add to block sum
							movd sum, mm6; // update sum of block
							//pop rbx;
							//pop rdi;
							//pop rsi;
						}
#endif
					}
				}
				if(sum<min)
				{
					min=sum;
					patch_x = i;
					patch_y = j;
				}
			}
		}
	}

	else if(pixel_format == YV12)
	{
		// may it should be implemented differently, with lesser weight of chroma (like MVTools)
		for(int j = ymin; j<ymax; j++)
		{
			for(int i = xmin; i<xmax; i++)
			{
				if(m_source[j*m_width+i]==0)continue; // not good patch source
				sum=0;
				for(int iter_y=MAX(-winysize, -y); iter_y<MIN(winysize, m_height-y); iter_y++)
				{
					source_y = j+iter_y;
					target_y = y+iter_y;

					unsigned char * tysrc = psrc + target_y*src_pitch;
					unsigned char * sysrc = psrc + source_y*src_pitch;
					unsigned char * tymark = m_mark + target_y*m_width;
					unsigned char * tysrcU = psrcU + (target_y>>1)*src_pitchUV;
					unsigned char * sysrcU = psrcU + (source_y>>1)*src_pitchUV;
					unsigned char * tysrcV = psrcV + (target_y>>1)*src_pitchUV;
					unsigned char * sysrcV = psrcV + (source_y>>1)*src_pitchUV;

					if (x-winxsize<0 || x+winxsize>m_width) // process border separately to process middle without checking (faster)
					{

						for(int iter_x=-winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							if(target_x<0||target_x>=m_width)continue;

							// it is the most time-comsuming part of code:
							if(tymark[target_x]==SOURCE) // compare
							{
							int temp_y = tysrc[target_x]-sysrc[source_x];
							int temp_u = tysrcU[(target_x>>1)]-sysrcU[(source_x>>1)];
							int temp_v = tysrcV[(target_x>>1)]-sysrcV[(source_x>>1)];

//							sum += temp_y*temp_y + temp_u*temp_u + temp_v*temp_v; // SSD
							sum += abs(temp_y) + abs(temp_u) + abs(temp_v); // SAD
							}
						}
					}
					else // middle
					{
#if (!ISSE)
						for(int iter_x=-winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							// it is the most time-comsuming part of code:
							int smark = -(int)(tymark[target_x]==SOURCE); // compare, remove jump for speed
							int temp_y = tysrc[target_x]-sysrc[source_x];
							int temp_u = tysrcU[(target_x>>1)]-sysrcU[(source_x>>1)];
							int temp_v = tysrcV[(target_x>>1)]-sysrcV[(source_x>>1)];

//							sum += temp_y*temp_y + temp_u*temp_u + temp_v*temp_v; // SSD
							sum += (abs(temp_y) + abs(temp_u) + abs(temp_v)) & smark; // SAD
						}
#else
						_asm
						{
							//push rsi;
							//push rdi;
							//push rbx;
							// assume winxsize is multiple 4
							mov rsi, sysrc;
							mov rdi, tysrc;
							mov eax, i;
							mov ecx, winxsize1;
							sub rax, rcx; // source_x = i + iter_x (start iter_x = -winxsize)
							mov edx, x;
							sub rdx, rcx; // target_x = x + iter_x
							shr rcx, 1; // count by 4 pixels
							mov rbx, tymark;
							pxor mm0, mm0; // constant 0
							pcmpeqd mm1, mm1; // constant two FFFFFFFF
							pxor mm2, mm2; // 0, will be our block sum as low word
							psrlq mm1, 32; // constant 00000000 FFFFFFFF hi dw only
align 16
startY:
							movd mm3, [rbx + rdx]; // read 4 pixel mark
							pcmpeqb mm3, mm0;// compare every byte mark with 0
							movd mm4, [rsi + rax]; // read 4 source pixels
							movd mm5, [rdi + rdx]; // read 4 target pixels
							pand mm3, mm1; // clear high
							pand mm4, mm3;// clear masked bytes
							pand mm5, mm3;// clear masked bytes
							psadbw mm4, mm5; // get  summ of absolute differenses SAD of 4 masked Y pixels
							paddw mm2, mm4; // add to block sum
							add rdx, 4; // next mark
							add rax, 4; // next 4 pixels
							dec rcx;
							jrcxz endY;
							jmp startY;
endY:
							mov rsi, sysrcU;
							mov rdi, tysrcU;
							mov eax, i;
							mov ecx, winxsize1;
							sub eax, ecx; // source_x = i + iter_x (start iter_x = -winxsize)
							sar eax, 1; //   /2
							mov edx, x;
							sub edx, ecx; // target_x = x + iter_x
							sar edx, 1; //    /2
							shr ecx, 1; // count by 4 pixels
							mov rbx, tymark;
							pxor mm0, mm0; // constant 0
							pcmpeqd mm1, mm1; // constant two FFFFFFFF
//							pxor mm2, mm2; // 0, will be our block sum as low word
							psrlq mm1, 32; // constant 00000000 FFFFFFFF
align 16
startU:
							movd mm3, [rbx + rdx*2]; // read 4 pixel mark
							pcmpeqb mm3, mm0;// compare every byte mark with 0
							movd mm4, [rsi + rax]; // read 4 source pixels, but use 2 only
							punpcklbw mm4, mm4; // 2 bytes to 2 words with duplicate bytes
							movd mm5, [rdi + rdx]; // read 4 target pixels, but use 2 only
							punpcklbw mm5, mm5; // 2 bytes to 2 words
							pand mm3, mm1; // clear high
							pand mm4, mm3;// clear masked bytes
							pand mm5, mm3;// clear masked bytes
							psadbw mm4, mm5; // get  summ of absolute differenses SAD of 2 masked U pixels
//							psrlw mm4, 1; // divide by 2 ? or 4 ?
							paddw mm2, mm4; // add to block sum
							add rdx, 2; // next mark
							add rax, 2; // next 2 pixels
							dec rcx;
							jrcxz endU;
							jmp startU;
endU:
							mov rsi, sysrcV;
							mov rdi, tysrcV;
							mov eax, i;
							mov ecx, winxsize1;
							sub rax, rcx; // source_x = i + iter_x (start iter_x = -winxsize)
							sar rax, 1; //   /2
							mov edx, x;
							sub rdx, rcx; // target_x = x + iter_x
							sar rdx, 1; //    /2
							shr rcx, 1; // count by 4 pixels
							mov rbx, tymark;
							pxor mm0, mm0; // constant 0
							pcmpeqd mm1, mm1; // constant two FFFFFFFF
//							pxor mm2, mm2; // 0, will be our block sum as low word
							psrlq mm1, 32; // constant 00000000 FFFFFFFF
align 16
startV:
							movd mm3, [rbx + rdx*2]; // read 4 pixel mark
							pcmpeqb mm3, mm0;// compare every byte mark with 0
							movd mm4, [rsi + rax]; // read 4 source pixels, but use 2 only
							punpcklbw mm4, mm4; // 2 bytes to 2 words with duplicate bytes
							movd mm5, [rdi + rdx]; // read 4 target pixels, but use 2 only
							punpcklbw mm5, mm5; // 2 bytes to 2 words
							pand mm3, mm1; // clear high
							pand mm4, mm3;// clear masked bytes
							pand mm5, mm3;// clear masked bytes
							psadbw mm4, mm5; // get  summ of absolute differenses SAD of 2 masked U pixels
//							psrlw mm4, 1; // divide by 2 ? or 4 ?
							paddw mm2, mm4; // add to block sum
							add rdx, 2; // next mark
							add rax, 2; // next 2 pixels
							dec rcx;
							jrcxz endV;
							jmp startV;
endV:
							movd mm6, sum;// get old sum of block
							punpcklwd mm2, mm0; // words with partial sum to doublewords, use low
							paddd mm6, mm2; // add to block sum
							movd sum, mm6; // update sum of block
							//pop rbx;
							//pop rdi;
							//pop rsi;
						}
#endif
					}
				}
				if(sum<min)
				{
					min=sum;
					patch_x = i;
					patch_y = j;
				}
			}
		}
	}
	else if(pixel_format == YUY2)
	{

		for(int j = ymin; j<ymax; j++)
		{
			for(int i = xmin; i<xmax; i++)
			{
				if(m_source[j*m_width+i]==0)continue; // not good patch source
				sum=0;

				for(int iter_y=MAX(-winysize, -y); iter_y<MIN(winysize, m_height-y); iter_y++)
				{
					source_y = j+iter_y;
					target_y = y+iter_y;

					unsigned char * tysrc = psrc + target_y*src_pitch;
					unsigned char * sysrc = psrc + source_y*src_pitch;
					unsigned char * tymark = m_mark + target_y*m_width;

					if (x-winxsize<0 || x+winxsize>m_width) // process border separately to process middle without checking (faster)
					{

						for(int iter_x=(-1)*winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							if(target_x<0||target_x>=m_width)continue;

							// it is the most time-comsuming part of code:
							if(tymark[target_x]==SOURCE) // compare
							{
								int tx4 = (target_x>>1)<<2; // mult 4
								int tU = *(tysrc + tx4 + 1);
								int tV = *(tysrc + tx4 + 3);
								int tY = *(tysrc + (target_x<<1));
								int sx4 = (source_x>>1)<<2; // mult 4
								int sY = *(sysrc + (source_x<<1));
								int sU = *(sysrc + sx4 + 1);
								int sV = *(sysrc + sx4 + 3);
								int temp_y = tY - sY;
								int temp_u = tU - sU;
								int temp_v = tV - sV;

//								sum += temp_r*temp_r + temp_g*temp_g + temp_b*temp_b; // SSD
								sum += (abs(temp_y) + abs(temp_u) + abs(temp_v)) ; // SAD
							}
						}
					}
					else // middle
					{
#if (1)
						for(int iter_x=-winxsize; iter_x<winxsize; iter_x++)
						{
							source_x = i+iter_x;
							target_x = x+iter_x;
							// it is the most time-comsuming part of code:
							int smark = -(int)(tymark[target_x]==SOURCE); // compare, remove jump for speed
							{
								int tx4 = (target_x>>1)<<2; // mult 4
								int tU = *(tysrc + tx4 + 1);
								int tV = *(tysrc + tx4 + 3);
								int tY = *(tysrc + (target_x<<1));
								int sx4 = (source_x>>1)<<2; // mult 4
								int sY = *(sysrc + (source_x<<1));
								int sU = *(sysrc + sx4 + 1);
								int sV = *(sysrc + sx4 + 3);
								int temp_y = tY - sY;
								int temp_u = tU - sU;
								int temp_v = tV - sV;

//								sum += (temp_r*temp_r + temp_g*temp_g + temp_b*temp_b) & smark; // SSD
								sum += (abs(temp_y) + abs(temp_u) + abs(temp_v)) & smark; // SAD
							}
						}
#else
						_asm
						{
							// not finished!, use YUV24 instead
							push rsi;
							push rdi;
							push rbx;
							// assume winxsize is multiple 2
							mov rsi, sysrc;
							mov rdi, tysrc;
							mov rax, i;
							mov rcx, winxsize1;
							sub rax, rcx; // source_x = i + iter_x (start iter_x = -winxsize)
							mov rdx, x;
							sub rdx, rcx; // target_x = x + iter_x
							mov rbx, tymark;
							pxor mm0, mm0; // constant 0
							pcmpeqw mm1, mm1; // constant two FFFFFFFF
							pxor mm2, mm2; // 0, will be our block sum as low word
							psrlw mm1, 8; // constant two 00FF00FF, i.e. Y mask
							push rbp;
align 16
startYUY2:
							movd mm3, [rbx + rdx]; // read 4 pixel mark, but will use 2 only
							punpcklbw mm3, mm0; // bytes to 4 words
//							punpcklwd mm3, mm0; // words to 2 doublewords
							pcmpeqd mm3, mm0;// compare every word mark with 0
							mov rbp, rax;
							shr rbp, 1; //    /2
							movd mm4, [rsi + rbp*4]; // read 2 source pixels by 2 bytes for Chroma, mult 4
							movd mm6, [rsi + rax*2]; // read 2 source pixels by 2 bytes for Luma

							mov rbp, rdx;
							shr rbp, 1; //    /2
							movq mm5, [rdi + rbp*4]; // read 4 target pixels by 2 bytes for chroma
							pand mm3, mm1; // final mask of mark
							pand mm4, mm3;// clear masked bytes
							pand mm5, mm3;// clear masked bytes
							psadbw mm4, mm5; // get  summ of absolute differenses SAD of 2 masked  pixels
							paddw mm2, mm4; // add to block sum
							add rdx, 2; // nex mark
							add rax, 2; // next 2 pixels
							dec rcx;
							jrcxz endYUY2;
							jmp startYUY2;
endYUY2:
							pop rbp;
							movd mm6, sum;// get old sum of block
							punpcklwd mm2, mm0; // words with partial sum to doublewords, use low
							paddd mm6, mm2; // add to block sum
							movd sum, mm6; // update sum of block
							pop rbx;
							pop rdi;
							pop rsi;
						}
#endif
					}
				}
				if(sum<min)
				{
					min=sum;
					patch_x = i;
					patch_y = j;
				}
			}
		}
	}


#if (ISSE)
	_asm emms;
#endif

	if (min == MIN_INITIAL)
		return false; // patch not found
	else
		return true; // found
}

/*********************************************************************/
bool inpainting::update(int target_x, int target_y, int source_x, int source_y, int confid)
{
	// apply patch

	if(pixel_format==RGB32 || pixel_format == RGBA)
	{

		unsigned int * intsrc = reinterpret_cast<unsigned int *>(psrc); // faster access
		int intsrc_pitch = src_pitch/4; // in int

		int x0,y0,x1,y1;
		for(int iter_y=MAX(-winysize, -target_y); iter_y<MIN(winysize, m_height-target_y); iter_y++)// add bound check - Fizick
		{
				y0 = source_y+iter_y;
				y1 = target_y + iter_y;

			for(int iter_x=MAX(-winxsize, -target_x); iter_x<MIN(winxsize, m_width-target_x); iter_x++)// add bound check
			{
				x0 = source_x+iter_x;
				x1 = target_x + iter_x;

				if(m_mark[y1*m_width+x1]!=SOURCE)
				{
					m_mark[y1*m_width+x1] = SOURCE; // now filled
					m_gray[y1*m_width+x1] = m_gray[y0*m_width+x0]; // inpaint the gray
					m_confid[y1*m_width+x1] = confid; // update the confidence
					*(intsrc + y1*intsrc_pitch + x1) = *(intsrc + y0*intsrc_pitch + x0);// inpaint the color and alpha
				}
			}
		}
	}
	else if (pixel_format==RGB24 || pixel_format==YUV24)
	{

		int x0,y0,x1,y1;
		for(int iter_y=MAX(-winysize, -target_y); iter_y<MIN(winysize, m_height-target_y); iter_y++)// add bound check - Fizick
		{
				y0 = source_y+iter_y;
				y1 = target_y + iter_y;

			for(int iter_x=MAX(-winxsize, -target_x); iter_x<MIN(winxsize, m_width-target_x); iter_x++)// add bound check
			{
				x0 = source_x+iter_x;
				x1 = target_x + iter_x;

				if(m_mark[y1*m_width+x1]!=SOURCE)
				{
					m_mark[y1*m_width+x1] = SOURCE; // now filled
					m_gray[y1*m_width+x1] = m_gray[y0*m_width+x0]; // inpaint the gray
					m_confid[y1*m_width+x1] = confid; // update the confidence
					*(psrc + y1*src_pitch + x1*3) = *(psrc + y0*src_pitch + x0*3);// inpaint the color B
					*(psrc + y1*src_pitch + x1*3+1) = *(psrc + y0*src_pitch + x0*3+1);// inpaint the color G
					*(psrc + y1*src_pitch + x1*3+2) = *(psrc + y0*src_pitch + x0*3+2);// inpaint the color R
				}
			}
		}
	}
	else if (pixel_format==YUY2)
	{

		int x0,y0,x1,y1;
		for(int iter_y=MAX(-winysize, -target_y); iter_y<MIN(winysize, m_height-target_y); iter_y++)// add bound check - Fizick
		{
				y0 = source_y+iter_y;
				y1 = target_y + iter_y;

			for(int iter_x=MAX(-winxsize, -target_x); iter_x<MIN(winxsize, m_width-target_x); iter_x++)// add bound check
			{
				x0 = source_x+iter_x;
				x1 = target_x + iter_x;

				if(m_mark[y1*m_width+x1]!=SOURCE)
				{
					m_mark[y1*m_width+x1] = SOURCE; // now filled
					m_confid[y1*m_width+x1] = confid; // update the confidence
					m_gray[y1*m_width+x1] = m_gray[y0*m_width+x0]; // inpaint the gray
					int x04 = (x0>>1)<<2; // mult 4
					int U = *(psrc + y0*src_pitch + x04 + 1);
					int V = *(psrc + y0*src_pitch + x04 + 3);
					int Y = *(psrc + y0*src_pitch + (x0<<1));
					int x14 = (x1>>1)<<2; // mult 4
					*(psrc + y1*src_pitch + (x1<<1)) = Y;
					*(psrc + y1*src_pitch + x14 + 1) = U;
					*(psrc + y1*src_pitch + x14 + 3) = V;
				}
			}
		}
	}
	else if (pixel_format==YV12)
	{

		int x0,y0,x1,y1;
		for(int iter_y=MAX(-winysize, -target_y); iter_y<MIN(winysize, m_height-target_y); iter_y++)// add bound check - Fizick
		{
				y0 = source_y+iter_y;
				y1 = target_y + iter_y;

			for(int iter_x=MAX(-winxsize, -target_x); iter_x<MIN(winxsize, m_width-target_x); iter_x++)// add bound check
			{
				x0 = source_x+iter_x;
				x1 = target_x + iter_x;

				if(m_mark[y1*m_width+x1]!=SOURCE)
				{
					m_mark[y1*m_width+x1] = SOURCE; // now filled
					m_confid[y1*m_width+x1] = confid; // update the confidence
					*(psrc + y1*src_pitch + x1) = *(psrc + y0*src_pitch + x0);// inpaint Y
					// gray is impainted as luma Y
					*(psrcU + (y1>>1)*src_pitchUV + (x1>>1)) = *(psrcU + (y0>>1)*src_pitchUV + (x0>>1));// inpaint the U
					*(psrcV + (y1>>1)*src_pitchUV + (x1>>1)) = *(psrcV + (y0>>1)*src_pitchUV + (x0>>1));// inpaint the V
				}
			}
		}
	}
	return true;
}

/*********************************************************************/
bool inpainting::TargetExist(void)
{
		for(int j= m_top; j<=m_bottom; j++)
			for(int i = m_left; i<= m_right; i++)
				if(m_mark[j*m_width+i]!=SOURCE)
					return true;
	return false;
}

/*********************************************************************/
void inpainting::UpdateBoundary(int i, int j)// just update the area near the changed patch. (+-2 pixels)
{
	int x, y;

	for(y = MAX(j -winysize-2,0); y< MIN(j+winysize+2,m_height); y++)
		for( x = MAX(i-winxsize-2,0); x<MIN(i+winxsize+2, m_width); x++)
		{
            if (m_mark[y*m_width+x]!=SOURCE)// was target or boundary and was not patched
			    m_mark[y*m_width+x] = TARGET;
		}

	for(y = MAX(j -winysize-2,0); y< MIN(j+winysize+2,m_height); y++)
		for( x = MAX(i-winxsize-2,0); x<MIN(i+winxsize+2, m_width); x++)
		{
			if(m_mark[y*m_width+x]==TARGET)
			{
				if(y==m_height-1||y==0||x==0||x==m_width-1
					|| m_mark[(y-1)*m_width+x]==SOURCE || m_mark[y*m_width+x-1]==SOURCE
					|| m_mark[y*m_width+x+1]==SOURCE || m_mark[(y+1)*m_width+x]==SOURCE)
				{

						m_mark[y*m_width+x] = BOUNDARY;
				}
			}
		}
}

/*********************************************************************/
int inpainting::UpdatePri(int i, int j) // just update the area near the changed patch. (+-3 pixels)
{
	int x,y;
	int max_pri_new = -1; // init as not valid
	for(y = MAX(j -winysize-3,0); y< MIN(j+winysize+3,m_height); y++)
		for( x = MAX(i-winxsize-3,0); x<MIN(i+winxsize+3, m_width); x++)
			if(m_mark[y*m_width+x] == BOUNDARY)
			{
				int pri = priority(x,y);
				m_pri[y*m_width+x] = pri;
				if (pri >= max_pri) // if new local pri is greater than old max in same block,
				{ // therefore there is no need in slow global search (Fizick)
					max_pri_new = pri; // get new max here
					max_pri = pri; // update new max here
					pri_x = x; // and location of the max
					pri_y = y;
				}
			}

	return max_pri_new; // valid if >= 0 only

}
