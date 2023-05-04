/*
 * Copyright (c) 2023 Xinqi Jin <jxqhhh@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/eval.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "avfilter.h"
#include "internal.h"
#include "video.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_wavelet.h>
#include <gsl/gsl_wavelet2d.h>
#include "RngStream.h"

enum Mode{
    Embedding = 3,
    Extraction = 4
};

typedef struct WatermarkContext {
    const AVClass *class;
    int keyFrameInterval;
    int numEmbeddedFrames;
    int numEmbeddedFramesTotal;
    int curFrameIdx;
    int frameIdx;
    float strength;
    int implicit;
    int seed;
    int regionSize;
    RngStream stream;
    int* indexOfWatermarkedFrames;
    int* watermarkedRegionRowOffset;
    int* watermarkedRegionColOffset;
    int mode;
    double referenceFrameLLHsum;
    double referenceFrameSVsum;
    double watermarkedFrameLLHsum;
    double watermarkedFrameSVsum;
    gsl_wavelet* wavelet;
    gsl_wavelet_workspace *work;
    char* outputFn;
    FILE* outputFile;
} WatermarkContext;

#define OFFSET(x) offsetof(WatermarkContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM


static const AVOption watermark_options[] = {
        { "keyFrameInterval", "set the interval between two consecutive IDR frames", OFFSET(keyFrameInterval), AV_OPT_TYPE_INT, {.i64 = 2}, 2, 5000000, FLAGS},
        {"numEmbeddedFrames", "set the num of frames embedded with the watermark in each segment", OFFSET(numEmbeddedFrames), AV_OPT_TYPE_INT, {.i64 = 1}, 0, 5000000, FLAGS},
        {"strength", "strength of the embedded watermark", OFFSET(strength), AV_OPT_TYPE_FLOAT, {.dbl=1.0}, 0.1, 10, FLAGS},
        {"seed", "seed for the random number generator", OFFSET(seed), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, FLAGS},
        {"implicitSVD", "whether to use implicit SVD or not", OFFSET(implicit), AV_OPT_TYPE_INT, {.i64=1}, 0, 1, FLAGS}, // 1 corresponds to implict SVD-based watermark embedding
        {"size", "the width of the region to be watermarked", OFFSET(regionSize), AV_OPT_TYPE_INT, {.i64=512}, 8, INT_MAX, FLAGS},
        {"mode", "the working mode (embedding or extraction?)", OFFSET(mode), AV_OPT_TYPE_INT, {.i64=3}, 3, 4, FLAGS},
        {"outputFn", "the name of the file to store the result", OFFSET(outputFn), AV_OPT_TYPE_STRING, .flags = FLAGS},
        { NULL }
};

static int
binary_logn (const int n)
{
    int ntest;
    int logn = 0;
    int k = 1;

    while (k < n)
    {
        k *= 2;
        logn++;
    }

    ntest = 1 << logn;

    if (n != ntest)
    {
        return -1;                /* n is not a power of 2 */
    }

    return logn;
}


AVFILTER_DEFINE_CLASS(watermark);

static int config_input(AVFilterLink *link)
{

    WatermarkContext *priv = link->dst->priv;
    priv->curFrameIdx = -1;
    priv->frameIdx = -1;
    priv->numEmbeddedFramesTotal = 0;

    if (priv->numEmbeddedFrames*2>priv->keyFrameInterval){
        printf("Invalid params: priv->numEmbeddedFrames*2>priv->keyFrameInterval\n");
        exit(1);
    }
    priv->indexOfWatermarkedFrames = (int*) malloc (sizeof(int)*priv->numEmbeddedFrames);
    priv->watermarkedRegionRowOffset = (int*) malloc (sizeof(int)*priv->numEmbeddedFrames);
    priv->watermarkedRegionColOffset = (int*) malloc (sizeof(int)*priv->numEmbeddedFrames);

    priv->stream = RngStream_CreateStream("main");
    unsigned long seeds[6];
    for (int i = 0; i < 6; i ++) {
        seeds[i] = priv->seed + i;
    }
    RngStream_SetSeed (priv->stream, seeds);

    if (link->w < priv->regionSize){
        priv->regionSize = link->w;
    }
    if (link->h < priv->regionSize){
        priv->regionSize = link->h;
    }

    priv->wavelet = gsl_wavelet_alloc (gsl_wavelet_haar, 2);
    priv->work = gsl_wavelet_workspace_alloc (priv->regionSize);

    if (binary_logn(priv->regionSize)==-1){
        printf("Error: regionSize is not a power of 2 .\n");
        exit(1);
    }

    priv->outputFile = fopen(priv->outputFn, "w");

    printf("Config input done\n");


    return 0;
}


static void AVFrameToArray (AVFrame* frame, gsl_matrix* mat, int rowOffset, int colOffset, int regionSize) {
    for (int i = 0; i < regionSize; i++) {
        for (int j = 0; j < regionSize; j++) {
            gsl_matrix_set(mat, i, j, frame->data[0][frame->linesize[0] * (i+rowOffset) + colOffset + j]);
        }
    }
    return;
}

static void ArrayToAVFrame (gsl_matrix* mat, AVFrame* frame, int rowOffset, int colOffset, int regionSize) {
    for (int i = 0; i < regionSize; i++) {
        for (int j = 0; j < regionSize; j++) {
            frame->data[0][frame->linesize[0] * (i+rowOffset) + colOffset + j] = gsl_matrix_get(mat, i, j);
        }
    }
    return;
}

static void watermark (AVFilterContext* ctx, AVFrame *frame, int rowOffset, int colOffset) {

    WatermarkContext *priv = ctx->priv;

    gsl_matrix* img = gsl_matrix_alloc(priv->regionSize, priv->regionSize);
    AVFrameToArray(frame, img, rowOffset, colOffset, priv->regionSize);

    // use the LH_{3} subband
    gsl_wavelet2d_nstransform_with_specified_level(priv->wavelet, img->data, img->tda, img->size1, img->size2, gsl_wavelet_forward, 3, priv->work);
    gsl_matrix_view LLHview = gsl_matrix_submatrix(img, 0, priv->regionSize/8, priv->regionSize/8, priv->regionSize/8);

    if (priv->implicit){
        gsl_matrix_scale(&LLHview.matrix, priv->strength);
    } else {
        gsl_matrix *mat = &(LLHview.matrix);
        gsl_vector *work = gsl_vector_alloc(priv->regionSize / 8);
        gsl_vector *S = gsl_vector_alloc(priv->regionSize / 8);
        gsl_matrix *V = gsl_matrix_alloc(priv->regionSize / 8, priv->regionSize / 8);

        gsl_set_error_handler_off();
        gsl_linalg_SV_decomp(mat, V, S, work);

        gsl_vector_scale(S, priv->strength);

        gsl_matrix_scale_columns(mat, S);
        gsl_matrix *result = gsl_matrix_alloc(priv->regionSize / 8, priv->regionSize / 8);
        gsl_blas_dgemm(CblasNoTrans, CblasTrans,
                       1.0, mat, V,
                       0.0, result);

        gsl_matrix_memcpy(mat, result);

        gsl_vector_free(work);
        gsl_vector_free(S);
        gsl_matrix_free(V);
        gsl_matrix_free(result);
    }

    gsl_wavelet2d_nstransform_with_specified_level(priv->wavelet, img->data, img->tda, img->size1, img->size2, gsl_wavelet_backward, 3, priv->work);

    ArrayToAVFrame(img, frame, rowOffset, colOffset, priv->regionSize);

    gsl_matrix_free(img);

}

static void computeSVsum (AVFilterContext* ctx, AVFrame *frame, int rowOffset, int colOffset, int isReferenceFrame) {

    WatermarkContext *priv = ctx->priv;

    gsl_matrix* img = gsl_matrix_alloc(priv->regionSize, priv->regionSize);
    AVFrameToArray(frame, img, rowOffset, colOffset, priv->regionSize);

    // use the LH_{3} subband
    gsl_wavelet2d_nstransform_with_specified_level(priv->wavelet, img->data, img->tda, img->size1, img->size2, gsl_wavelet_forward, 3, priv->work);
    gsl_matrix_view LLHview = gsl_matrix_submatrix(img, 0, priv->regionSize/8, priv->regionSize/8, priv->regionSize/8);


    gsl_matrix *mat = &(LLHview.matrix);

    if(isReferenceFrame) {
        priv->referenceFrameLLHsum = 0;
        for (int i = 0; i < priv->regionSize/8; i ++){
            for (int j = 0; j < priv->regionSize/8; j ++){
                priv->referenceFrameLLHsum += fabs(gsl_matrix_get(mat, i, j));
            }
        }
    }else{
        priv->watermarkedFrameLLHsum = 0;
        for (int i = 0; i < priv->regionSize/8; i ++){
            for (int j = 0; j < priv->regionSize/8; j ++){
                priv->watermarkedFrameLLHsum += fabs(gsl_matrix_get(mat, i, j));
            }
        }
    }

    gsl_vector *work = gsl_vector_alloc(priv->regionSize / 8);
    gsl_vector *S = gsl_vector_alloc(priv->regionSize / 8);
    gsl_matrix *V = gsl_matrix_alloc(priv->regionSize / 8, priv->regionSize / 8);

    gsl_set_error_handler_off();
    gsl_linalg_SV_decomp(mat, V, S, work);

    if(isReferenceFrame) {
        priv->referenceFrameSVsum = gsl_vector_sum(S);
    }else{
        priv->watermarkedFrameSVsum = gsl_vector_sum(S);
    }

    gsl_vector_free(work);
    gsl_vector_free(S);
    gsl_matrix_free(V);
    gsl_matrix_free(img);

}

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *avctx = link->dst;
    AVFilterLink *outlink = avctx->outputs[0];

    WatermarkContext *priv = avctx->priv;
    priv->curFrameIdx = (priv->curFrameIdx + 1) % priv->keyFrameInterval;
    priv->frameIdx ++;

    int interval = priv->keyFrameInterval / priv->numEmbeddedFrames;
    if (priv->curFrameIdx==0) {
        for (int i = 0; i < priv->numEmbeddedFrames; i ++) {
            priv->indexOfWatermarkedFrames[i] = interval*i + RngStream_RandInt(priv->stream, 1, interval-1);
            priv->watermarkedRegionRowOffset[i] = RngStream_RandInt(priv->stream, 0, link->h - priv->regionSize);
            priv->watermarkedRegionColOffset[i] = RngStream_RandInt(priv->stream, 0, link->w - priv->regionSize);
        }
    }


    if (priv->mode==Embedding) {

        int embeddingCurFrame = 0;
        int rowOffset = -1;
        int colOffset = -1;

        for (int i = 0; i < priv->numEmbeddedFrames; i++) {
            if (priv->indexOfWatermarkedFrames[i] == priv->curFrameIdx) {
                embeddingCurFrame = 1;
                rowOffset = priv->watermarkedRegionRowOffset[i];
                colOffset = priv->watermarkedRegionColOffset[i];
                break;
            }
        }


        if (embeddingCurFrame) {
            fprintf(priv->outputFile, "%d %d %d\n", priv->frameIdx, rowOffset, colOffset);
            printf("Embedding frame=%d rowOffset=%d colOffset=%d\n", priv->curFrameIdx, rowOffset, colOffset);
            AVFrame *out;
            out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
            if (!out) {
                av_frame_free(&in);
                return AVERROR(ENOMEM);
            }
            av_frame_copy_props(out, in);
            av_frame_copy(out, in);
            av_frame_free(&in);

            watermark(avctx, out, rowOffset, colOffset);
            priv->numEmbeddedFramesTotal++;

            return ff_filter_frame(outlink, out);
        } else {
            return ff_filter_frame(outlink, in);
        }
    } else {
        int referenceFrame = 0;
        int watermarkedFrame = 0;
        int rowOffset = -1;
        int colOffset = -1;

        for (int i = 0; i < priv->numEmbeddedFrames; i++) {
            if (priv->indexOfWatermarkedFrames[i] == priv->curFrameIdx+1) {
                referenceFrame = 1;
                rowOffset = priv->watermarkedRegionRowOffset[i];
                colOffset = priv->watermarkedRegionColOffset[i];
                break;
            }
            if (priv->indexOfWatermarkedFrames[i] == priv->curFrameIdx) {
                watermarkedFrame = 1;
                rowOffset = priv->watermarkedRegionRowOffset[i];
                colOffset = priv->watermarkedRegionColOffset[i];
                break;
            }
        }

        if (referenceFrame==1) {
            computeSVsum(avctx, in, rowOffset, colOffset, 1);
        } else if (watermarkedFrame==1) {
            computeSVsum(avctx, in, rowOffset, colOffset, 0);
            if (priv->watermarkedFrameSVsum<priv->referenceFrameSVsum) {
                fprintf(priv->outputFile, "Detected bit is 0 using SVD\n");
            } else {
                fprintf(priv->outputFile, "Detected bit is 1 using SVD\n");
            }
            if (priv->watermarkedFrameLLHsum<priv->referenceFrameLLHsum) {
                fprintf(priv->outputFile, "Detected bit is 0 using DWT\n");
            } else {
                fprintf(priv->outputFile, "Detected bit is 1 using DWT\n");
            }
        }
        return ff_filter_frame(outlink, in);
    }

}

static const enum AVPixelFormat pixel_fmts_watermarkfilt[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NONE
};

static const AVFilterPad avfilter_vf_watermark_inputs[] = {
        {
                .name = "default",
                .type = AVMEDIA_TYPE_VIDEO,
                .filter_frame = filter_frame,
                .config_props = config_input,
        }
};

static const AVFilterPad avfilter_vf_watermark_outputs[] = {
        {
                .name = "default",
                .type = AVMEDIA_TYPE_VIDEO,
        }
};

AVFilter ff_vf_watermark= {
        .name = "watermark",
        .description = NULL_IF_CONFIG_SMALL("Video Watermarking."),
        .priv_size = sizeof(WatermarkContext),
        .priv_class = &watermark_class,
        FILTER_INPUTS(avfilter_vf_watermark_inputs),
        FILTER_OUTPUTS(avfilter_vf_watermark_outputs),
        FILTER_PIXFMTS_ARRAY(pixel_fmts_watermarkfilt),
        .flags = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
};