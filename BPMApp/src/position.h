/*
 * position.cpp
 *
 * Authors: Lucas Russo
 *
 * Created Mar. 17, 2020
 */

#include <inttypes.h>

/* Button values. */
typedef struct
{
    int32_t A;
    int32_t B;
    int32_t C;
    int32_t D;
} ABCDRow_t;

#define ABCD_ROW ABCDRow_t

/* Computed X, Y values in nm, S in arbitrary units. */
typedef struct
{
    double X;
    double Y;
    double Q;
    double S;
} XYQSRow_t;

#define XYQS_ROW XYQSRow_t

/* K factors, in nm. */
typedef struct
{
    uint32_t KX;
    uint32_t KY;
    uint32_t KQ;
    uint32_t KSUM;
} KFactors_t;

#define K_FACTORS KFactors_t

/* Postion offset, in nm. */
typedef struct
{
    int32_t XOFFSET;
    int32_t YOFFSET;
    int32_t QOFFSET;
} PosOffsets_t;

#define POS_OFFSETS PosOffsets_t

/* Calculates the reciprocal */
double Reciprocal(double X);

/* Wrapper function to ABCD to WYQS with different methods */
void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, K_FACTORS *K, POS_OFFSETS *OFFSETS,
        int Count, bool Diagonal = true, bool PartialDelta = true);

/* Converts Count rows of ABCD button data into XYQS position and intensity
 * data via the configured conversion function. */
/* void ABCDtoXYQS(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, int Count); */
void ABCDtoXYQSStd(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, K_FACTORS *K, POS_OFFSETS *OFFSETS,
        int Count, bool Diagonal = true);

/* Converts Count rows of ABCD button data into XYQS position and intensity
 * using Partial Delta-over-Sum algorithm */
void ABCDtoXYQSPartial(const ABCD_ROW *ABCD, XYQS_ROW *XYQS, K_FACTORS *K, POS_OFFSETS *OFFSETS,
        int Count, bool Diagonal = true);
