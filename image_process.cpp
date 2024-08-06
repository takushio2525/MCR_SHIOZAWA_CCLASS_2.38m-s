//------------------------------------------------------------------//
//Supported MCU:   RZ/A1H
//File Contents:   Image Processing API ( Source file )
//Version number:  Ver.1.01
//Date:            2019.02.02
//Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//------------------------------------------------------------------//

//------------------------------------------------------------------//
//Include
//------------------------------------------------------------------//
#include <math.h>
#include "mbed.h"
#include "image_process.h"

/*******************************/
/* Function Map                */
/********************************
* ImageCopy
* Extraction_Brightness
* ImageReduction
* Binarization
* Percentile_Method
* DiscriminantAnalysis_Method
* Image_part_Extraction
* Standard_Deviation
* Covariance
* Judgement_ImageMatching
* PatternMatching_process
********************************/

//******************************************************************//
// Image process functions
//*******************************************************************/
//------------------------------------------------------------------//
// ImageCopy function
//------------------------------------------------------------------//
void ImageCopy( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, int Frame )
{
    static int  counter = 0;
    static int  X, Y;
    int         HW_T;//HW Twice

    HW_T  = HW + HW;

    switch( counter++ ) {
    case 0:
        // Top or Bottom field
        for( Y = Frame; Y < ( VW / 2 ); Y+=2 ){
            for( X = 0; X < HW_T; X++ ){
                BuffAddrOut[ ( Y * HW_T ) + X ] = BuffAddrIn[ ( Y * HW_T ) + X ];
            }
        }
        break;
    case 1:
        // Top or Bottom field
        for(          ; Y < VW; Y+=2 ){
            for( X = 0; X < HW_T; X++ ){
                BuffAddrOut[ ( Y * HW_T ) + X ] = BuffAddrIn[ ( Y * HW_T ) + X ];
            }
        }
        //Frame Change
        if( Frame == 0 ) Frame = 1;
        else             Frame = 0;
        for( Y = Frame; Y < VW; Y+=2 ){
            for( X = 0; X < HW_T; X+=2 ){
                BuffAddrOut[ ( Y * HW_T ) + ( X + 0 ) ] = 0;
                BuffAddrOut[ ( Y * HW_T ) + ( X + 1 ) ] = 128;
            }
        }
        counter = 0;
        break;
    default:
        break;
    }
}

//------------------------------------------------------------------//
// Extraction Brightness function
//------------------------------------------------------------------//
void Extraction_Brightness( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, int Frame )
{
    static int  conter = 0;

    int X, Y;
    int Px;
    int HW_T;//HW Twice

    HW_T  = HW + HW;

    switch( conter++ ) {
    case 0:
        //Pixel Interpolation ( Top or Bottom )
        for( Y = Frame; Y < ( VW / 2 ); Y+=2 ){
            for( X = 0, Px = 0; X < HW_T; X+=2, Px++  ){
                BuffAddrOut[ ( Y * HW ) + Px ] = BuffAddrIn[ ( Y * HW_T ) + X ];
            }
        }
        //Frame Change
        if( Frame == 0 ) Frame = 1;
        else             Frame = 0;
        //Bilinear Interpolation Method
        for( Y = Frame; Y < ( VW / 2 ); Y+=2 ){
            for( X = 0, Px = 0; X < HW_T; X+=2, Px++  ){
                if( Y <= 0 ) {
                    BuffAddrOut[ ( Y * HW ) + Px ] = BuffAddrOut[ ( (Y+1) * HW ) + Px ];
                } else if( ( ( VW / 2 ) - 1 ) > Y && Y > 0 ) {
                    BuffAddrOut[ ( Y * HW ) + Px ] = ( BuffAddrOut[ ( (Y-1) * HW ) + Px ] * (double)0.5 ) + ( BuffAddrOut[ ( (Y+1) * HW ) + Px ] * (double)0.5 );
                } else if( Y >= ( ( VW / 2 ) - 1 ) ) {
                    BuffAddrOut[ ( Y * HW ) + Px ] = BuffAddrOut[ ( (Y-1) * HW ) + Px ];
                }
            }
        }
        break;
    case 1:
        //Pixel Interpolation ( Top or Bottom )
        for( Y = ( VW / 2 ) + Frame; Y < VW; Y+=2 ){
            for( X = 0, Px = 0; X < HW_T; X+=2, Px++  ){
                BuffAddrOut[ ( Y * HW ) + Px ] = BuffAddrIn[ ( Y * HW_T ) + X ];
            }
        }
        //Frame Change
        if( Frame == 0 ) Frame = 1;
        else             Frame = 0;
        //Bilinear Interpolation Method
        for( Y = ( VW / 2 ) + Frame; Y < VW; Y+=2 ){
            for( X = 0, Px = 0; X < HW_T; X+=2, Px++  ){
                if( Y <= 0 ) {
                    BuffAddrOut[ ( Y * HW ) + Px ] = BuffAddrOut[ ( (Y+1) * HW ) + Px ];
                } else if( ( VW - 1 ) > Y && Y > 0 ) {
                    BuffAddrOut[ ( Y * HW ) + Px ] = ( BuffAddrOut[ ( (Y-1) * HW ) + Px ] * (double)0.5 ) + ( BuffAddrOut[ ( (Y+1) * HW ) + Px ] * (double)0.5 );
                } else if( Y >= ( VW - 1 ) ) {
                    BuffAddrOut[ ( Y * HW ) + Px ] = BuffAddrOut[ ( (Y-1) * HW ) + Px ];
                }
            }
        }
        conter = 0;
        break;
    default:
        break;
    }
}

//------------------------------------------------------------------//
// Image Reduction function
// Parcent : 0.0 - 1.0
//------------------------------------------------------------------//
void ImageReduction( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, double Percent )
{
    static int      conter = 0;
    static int      Y;

    int             X;
    int             HW_N;
    long            Nx, Ny;
    long            NxBuff, NyBuff = -1;
    unsigned int    BuffAddrData;
    double long     Sx, Sy;

    NxBuff  = -1;
    Sx = Sy = Percent;
    HW_N    = HW * Percent;

    //Under 100%
    if( Percent <= 1 ) {
        switch( conter++ ) {
        case 0:
            for( Y = 0; Y < ( VW / 2 ); Y++ ){
                //Affine Transformation Y-axis
                Ny = ( Sy * Y );
                for( X = 0; X < HW; X++ ){
                    //Affine Transformation X-axis
                    Nx = ( Sx * X );
                    if( NxBuff == Nx ) {
                        BuffAddrData  = BuffAddrOut[ ( Ny * HW_N ) + Nx ];
                        BuffAddrData += BuffAddrIn[ ( Y * HW ) + X ];
                        BuffAddrData /= 2;
                        BuffAddrOut[ ( Ny * HW_N ) + Nx ] = BuffAddrData;
                    } else {
                        NxBuff = Nx;
                        BuffAddrOut[ ( Ny * HW_N ) + Nx ] = BuffAddrIn[ ( Y * HW ) + X ];
                    }
                }
                if( NyBuff == Ny ) {
                    for( X = 0; X < HW_N; X++ ){
                        BuffAddrData  = BuffAddrOut[ (  Ny      * HW_N ) + X ];//Now line
                        BuffAddrData += BuffAddrOut[ ( (Ny - 1) * HW_N ) + X ];//Before now line
                        BuffAddrData /= 2;
                        BuffAddrOut[ ( Ny * HW_N ) + X ] = BuffAddrData;
                    }
                } else {
                    NyBuff = Ny;
                }
            }
            break;
        case 1:
            for(      ; Y < VW; Y++ ){
                //Affine Transformation Y-axis
                Ny = ( Sy * Y );
                for( X = 0; X < HW; X++ ){
                    //Affine Transformation X-axis
                    Nx = ( Sx * X );
                    if( NxBuff == Nx ) {
                        BuffAddrData  = BuffAddrOut[ ( Ny * HW_N ) + Nx ];
                        BuffAddrData += BuffAddrIn[ ( Y * HW ) + X ];
                        BuffAddrData /= 2;
                        BuffAddrOut[ ( Ny * HW_N ) + Nx ] = BuffAddrData;
                    } else {
                        NxBuff = Nx;
                        BuffAddrOut[ ( Ny * HW_N ) + Nx ] = BuffAddrIn[ ( Y * HW ) + X ];
                    }
                }
                if( NyBuff == Ny ) {
                    for( X = 0; X < HW_N; X++ ){
                        BuffAddrData  = BuffAddrOut[ (  Ny      * HW_N ) + X ];//Now line
                        BuffAddrData += BuffAddrOut[ ( (Ny - 1) * HW_N ) + X ];//Before now line
                        BuffAddrData /= 2;
                        BuffAddrOut[ ( Ny * HW_N ) + X ] = BuffAddrData;
                    }
                } else {
                    NyBuff = Ny;
                }
            }
            conter = 0;
            break;
        default:
            break;
        }
    }
}

//------------------------------------------------------------------//
// Binarization process Function
//------------------------------------------------------------------//
void Binarization( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, int threshold )
{
    int     i;
    int     items;

    items = HW * VW;

    for( i = 0; i < items; i++ ) {
        if( BuffAddrIn[i] >= threshold ) BuffAddrOut[i] = 1;
        else                             BuffAddrOut[i] = 0;
    }
}

//------------------------------------------------------------------//
// Percentile Method Function
//------------------------------------------------------------------//
int Percentile_Method( unsigned char *BuffAddrIn, int HW, int VW, int percent )
{
//    #define DEBUG_PM  //ON
    long    items, data;
    int     i;
    int     Hist[256];
    int     Threshold;

    //Total value of histogram
    items = HW * VW;

    //Histogram
    for( i = 0; i < 256;   i++ ) { Hist[ i ] = 0; }
    for( i = 0; i < items; i++ ) { Hist[ BuffAddrIn[i] ] += 1; }

#ifdef DEBUG_PM
    //Display histogram Data
    for( i = 0; i < 256; i++ ){
        pc.printf( "%d,", Hist[i] );
    }
    pc.printf("\n\rEnd\n\r");
#endif

    //Total value of rate
    Threshold = ( items * percent ) / 100;

    //Threshold value
    for( data = 0, i = 255; i > 0; i-- ){
        data += Hist[ i ];
        if( data > Threshold ) { Threshold = i; break; }
    }

    return Threshold;
}

//------------------------------------------------------------------//
// DiscriminantAnalysis_Method
//------------------------------------------------------------------//
int DiscriminantAnalysis_Method( unsigned char *BuffAddrIn, int HW, int VW )
{
//#define DEBUG_DAM  //ON
volatile long   w1, w2;
volatile long   m1, m2;
volatile long   data, data1, data2, data_buff;
         long   items;
         int    i, j;
         int    Hist[256];
         int    Threshold;

    data_buff = 0;

    //Total value of histogram
    items = HW * VW;

    //Histogram
    for( i = 0; i < 256;   i++ ) { Hist[ i ] = 0; }
    for( i = 0; i < items; i++ ) { Hist[ BuffAddrIn[i] ] += 1; }

    for( i = 0; i < 256; i++ ) {
        w1 = m1 = w2 = m2 = data = 0;
        //Black Pixel
        for( j = 0; j <   i; j++ ) { w1 += Hist[ j ]; m1 += ( j + 1 ) * Hist[ j ]; }
        m1 /= w1;
        //White Pixel
        for( j = i; j < 256; j++ ) { w2 += Hist[ j ]; m2 += ( j + 1 ) * Hist[ j ]; }
        m2 /= w2;
        //Threshold value
        data1 = w1 * w2 * ( ( ( m1 - m2) * ( m1 - m2) ) / 10 );
        data2 = ( ( w1 + w2 ) * ( w1 + w2 ) ) / 10;
        data  = data1 / data2;
#ifdef DEBUG_DAM
        pc.printf( "w1=%4ld, m1=%3ld, w2=%4ld, m2=%3ld, data=%4ld, t=%3d\n\r", w1, m1, w2, m2, data, i );
#endif
        if( data >= data_buff) {
            data_buff = data;
            Threshold = i;
        }
    }
#ifdef DEBUG_DAM
    pc.printf( "Threshold = %3d\n\r", Threshold );
    while( true );
#endif

    return Threshold;
}

//******************************************************************//
// Mark detect functions
//*******************************************************************/
//------------------------------------------------------------------//
// Extract_Image
//------------------------------------------------------------------//
void Image_part_Extraction( unsigned char *BuffAddrIn, int HW, int VW,
                            int CutPointX, int CutPointY, unsigned char *BuffAddrOut, int Xsize, int Ysize )
{
    int     X, Y;

    (void)VW;
    
    for( Y = 0; Y < Ysize; Y++ ) {
        for( X = 0; X < Xsize; X++ ) {
            BuffAddrOut[ X + ( Y * Xsize ) ] = BuffAddrIn[ ( ( Y + CutPointY ) * HW ) + ( X + CutPointX ) ];
       }
    }
}

//------------------------------------------------------------------//
// Standard deviation
//------------------------------------------------------------------//
double Standard_Deviation( unsigned char *data, double *Devi, int Xsize, int Ysize )
{
    int         i;
    int         items;
    double      iRet_A, iRet_C, iRet_D;

    items = Xsize * Ysize;

    /* A 合計値　平均化 */
    iRet_A = 0;
    for( i = 0; i < items; i++ ) {
        iRet_A += data[i];
    }
    iRet_A /= items;

    /* B 偏差値 */
    for( i = 0; i < items; i++ ) {
        Devi[i] = data[i] - iRet_A;
    }

    /* C 分散 */
    iRet_C = 0;
    for( i = 0; i < items; i++ ) {
        iRet_C += ( Devi[i] * Devi[i] );
    }
    iRet_C /= items;

    /* D 標準偏差 */
    iRet_D = sqrt( iRet_C );

    return iRet_D;
}

//------------------------------------------------------------------//
// Covariance
//------------------------------------------------------------------//
double Covariance( double *Devi_A, double *Devi_B, int Xsize, int Ysize )
{
    int     i;
    int         items;
    double  iRet, iRet_buff;

    items = Xsize * Ysize;

    iRet = 0;
    for( i = 0; i < items; i++ ) {
        iRet_buff = Devi_A[i] * Devi_B[i];
        iRet     += iRet_buff;
    }
    iRet /= items;

    return iRet;
}

//------------------------------------------------------------------//
// Judgement_ImageMatching
//------------------------------------------------------------------//
int Judgement_ImageMatching( double Covari, double SDevi_A, double SDevi_B )
{
    int     iRet;

    iRet  = ( Covari * 100 ) / ( SDevi_A * SDevi_B );

    return iRet;
}

//------------------------------------------------------------------//
// Pattern Matching process
//------------------------------------------------------------------//
void PatternMatching_process( unsigned char *BuffAddrIn, int HW, int VW,
                              ImagePartPattern *Temp, int Xs, int Xe, int Ys, int Ye )
{
    ImagePartPattern    NowImage;
    volatile int        x, y;
    volatile int        retJudge;
    volatile double     retCovari;

    Temp->p = 0;
    for( y = Ys; y <= Ye; y++ ) {
        for( x = Xs; x <= Xe; x++ ) {
            Image_part_Extraction( BuffAddrIn, HW, VW, x, y, NowImage.binary, Temp->w, Temp->h );
            NowImage.sdevi = Standard_Deviation( NowImage.binary, NowImage.devi, Temp->w, Temp->h);
            retCovari      = Covariance( Temp->devi, NowImage.devi, Temp->w, Temp->h );
            retJudge       = 0;
            retJudge       = Judgement_ImageMatching( retCovari, Temp->sdevi, NowImage.sdevi );
            if( 100 >= retJudge && retJudge > Temp->p ) {
                Temp->x = x;
                Temp->y = y;
                Temp->p = retJudge;
            }
        }
    }
}

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
