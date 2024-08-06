//------------------------------------------------------------------//
//Supported MCU:   RZ/A1H
//File Contents:   Image Processing API ( Header file )
//Version number:  Ver.1.01
//Date:            2019.02.02
//Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//------------------------------------------------------------------//
//Struct
//------------------------------------------------------------------//
typedef struct {
    volatile int    p;                  //percent
    volatile int    x;                  //Point X
    volatile int    y;                  //Point Y
    volatile double sdevi;              //Standard_Deviation
    double          devi[100];          //Deviation
    unsigned char   binary[100];        //Binary
    volatile int    w;                  //Binary Width pixel
    volatile int    h;                  //Binary Height pixel
} ImagePartPattern;

//------------------------------------------------------------------//
//Prototype( Image process )
//------------------------------------------------------------------//
void ImageCopy( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, int Frame );
void Extraction_Brightness( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, int Frame );
void ImageReduction( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, double Percent );
void Binarization( unsigned char *BuffAddrIn, int HW, int VW, unsigned char *BuffAddrOut, int threshold );
int Percentile_Method( unsigned char *BuffAddrIn, int HW, int VW, int percent );
int DiscriminantAnalysis_Method( unsigned char *BuffAddrIn, int HW, int VW );

//------------------------------------------------------------------//
//Prototype( Mark detection process )
//------------------------------------------------------------------//
void Image_part_Extraction( unsigned char *BuffAddrIn, int HW, int VW,
                            int CutPointX, int CutPointY, unsigned char *BuffAddrOut, int Xsize, int Ysize );
double Standard_Deviation( unsigned char *data, double *Devi, int Xsize, int Ysize );
double Covariance( double *Devi_A, double *Devi_B, int Xsize, int Ysize );
int Judgement_ImageMatching( double Covari, double SDevi_A, double SDevi_B );
void PatternMatching_process( unsigned char *BuffAddrIn, int HW, int VW,
                              ImagePartPattern *Temp, int Xs, int Xe, int Ys, int Ye );
//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
