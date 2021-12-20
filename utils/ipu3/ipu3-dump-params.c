/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ipu3-dump-params - Display IPU3 parameters buffer from a binary dump
 *
 * Copyright 2021 Jean-Michel Hautbois <jeanmichel.hautbois@ideasonboard.com>
 */
#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/intel-ipu3.h>

static void usage(const char *argv0)
{
	printf("Usage: %s input-file\n", basename(argv0));
	printf("Display the IPU3 parameters buffer\n");
}

static void displayGrid(struct ipu3_uapi_grid_config *grid, const char *gridName)
{
	fprintf(stderr, "Configured %s grid [%d,%d]x[%d,%d] starting at (%d, %d)\n",
		gridName,
		grid->width,
		grid->block_width_log2,
		grid->height,
		grid->block_height_log2,
		grid->x_start,
		grid->y_start & ~IPU3_UAPI_GRID_Y_START_EN);
	fprintf(stderr, "Grid size is (%d x %d)\n",
		grid->width << grid->block_width_log2,
		grid->height << grid->block_height_log2);
}

static void displayBNR(struct ipu3_uapi_params *params)
{
	struct ipu3_uapi_bnr_static_config bnr = params->acc_param.bnr;
	fprintf(stderr, "WB gains: (gr: %u, r: %u, gb: %u, b: %u)\n",
		bnr.wb_gains.gr, bnr.wb_gains.r,
		bnr.wb_gains.gb, bnr.wb_gains.b);
	fprintf(stderr, "WB gains thresholds: (gr: %u, r: %u, gb: %u, b: %u)\n",
		bnr.wb_gains_thr.gr, bnr.wb_gains_thr.r,
		bnr.wb_gains_thr.gb, bnr.wb_gains_thr.b);
	fprintf(stderr, "Optical window center: (%d, %d) column size is %u\n",
		bnr.opt_center.x_reset, bnr.opt_center.y_reset, bnr.column_size);
	fprintf(stderr, "Noise model coefficients that controls noise threshold:\n");
	fprintf(stderr, "Free coefficient (cf): %u, Gain coefficient(cg): %u\n",
		bnr.thr_coeffs.cf, bnr.thr_coeffs.cg);
	fprintf(stderr, "Intensity coefficient(ci): %u, Normalization shift value for r^2 calculation(r_nf): %u\n",
		bnr.thr_coeffs.ci, bnr.thr_coeffs.r_nf);
	fprintf(stderr, "Lens shading gain approximations: (gr: %u, r: %u, gb: %u, b: %u)\n",
		bnr.thr_ctrl_shd.gr, bnr.thr_ctrl_shd.r,
		bnr.thr_ctrl_shd.gb, bnr.thr_ctrl_shd.b);
}

static void displayAfFilter(struct ipu3_uapi_params *params)
{
	struct ipu3_uapi_af_filter_config filter = params->acc_param.af.filter_config;
	fprintf(stderr, "Configured af filter\ny1 => \n(%u, %u, %u, %u\n %u, %u, %u, %u\n %u, %u, %u, %u)\n vector: %x - normalization factor: %u\n",
		filter.y1_coeff_0.a1, filter.y1_coeff_0.a2, filter.y1_coeff_0.a3, filter.y1_coeff_0.a4,
		filter.y1_coeff_1.a5, filter.y1_coeff_1.a6, filter.y1_coeff_1.a7, filter.y1_coeff_1.a8,
		filter.y1_coeff_2.a9, filter.y1_coeff_2.a10, filter.y1_coeff_2.a11, filter.y1_coeff_2.a12,
		filter.y1_sign_vec, filter.nf.y1_nf);
	fprintf(stderr, "y2 => \n(%u, %u, %u, %u\n %u, %u, %u, %u\n %u, %u, %u, %u)\n vector: %x - normalization factor: %u\n",
		filter.y2_coeff_0.a1, filter.y2_coeff_0.a2, filter.y2_coeff_0.a3, filter.y2_coeff_0.a4,
		filter.y2_coeff_1.a5, filter.y2_coeff_1.a6, filter.y2_coeff_1.a7, filter.y2_coeff_1.a8,
		filter.y2_coeff_2.a9, filter.y2_coeff_2.a10, filter.y2_coeff_2.a11, filter.y2_coeff_2.a12,
		filter.y2_sign_vec, filter.nf.y2_nf);
	fprintf(stderr, "Channels coefficients: (gr: %u, r: %u, gb: %u, b: %u)\n",
		filter.y_calc.y_gen_rate_gr, filter.y_calc.y_gen_rate_r,
		filter.y_calc.y_gen_rate_gb, filter.y_calc.y_gen_rate_b);
}

static void displayLinLut(struct ipu3_uapi_params *params)
{
	struct ipu3_uapi_isp_lin_vmem_params linlut = params->lin_vmem_params;

	/* Gr */
	fprintf(stderr, "Linearization look-up table for Gr channel interpolation:\n");
	fprintf(stderr, "{ ");
	for (unsigned int index = 0; index < IPU3_UAPI_LIN_LUT_SIZE; index++) {
		if (index % (IPU3_UAPI_LIN_LUT_SIZE / 8) == 0)
			fprintf(stderr, "\n");
		fprintf(stderr, " %d,", linlut.lin_lutlow_gr[index]);
	}
	fprintf(stderr, "\b }\n");

	/* R */
	fprintf(stderr, "\nLinearization look-up table for R channel interpolation:\n");
	fprintf(stderr, "{ ");
	for (unsigned int index = 0; index < IPU3_UAPI_LIN_LUT_SIZE; index++) {
		if (index % (IPU3_UAPI_LIN_LUT_SIZE / 8) == 0)
			fprintf(stderr, "\n");
		fprintf(stderr, " %d,", linlut.lin_lutlow_r[index]);
	}
	fprintf(stderr, "\b }\n");

	/* Gb */
	fprintf(stderr, "\nLinearization look-up table for Gb channel interpolation:\n");
	fprintf(stderr, "{ ");
	for (unsigned int index = 0; index < IPU3_UAPI_LIN_LUT_SIZE; index++) {
		if (index % (IPU3_UAPI_LIN_LUT_SIZE / 8) == 0)
			fprintf(stderr, "\n");
		fprintf(stderr, " %d,", linlut.lin_lutlow_gb[index]);
	}
	fprintf(stderr, "\b }\n");

	/* B */
	fprintf(stderr, "\nLinearization look-up table for B channel interpolation:\n");
	fprintf(stderr, "{ ");
	for (unsigned int index = 0; index < IPU3_UAPI_LIN_LUT_SIZE; index++) {
		if (index % (IPU3_UAPI_LIN_LUT_SIZE / 8) == 0)
			fprintf(stderr, "\n");
		fprintf(stderr, " %d,", linlut.lin_lutlow_b[index]);
	}
	fprintf(stderr, "\b }\n");
}

int main(int argc, char *argv[])
{
	int in_fd;
	int ret = 0;
	struct ipu3_uapi_params params;

	if (argc != 2) {
		usage(argv[0]);
		return 1;
	}

	in_fd = open(argv[1], O_RDONLY);
	if (in_fd == -1) {
		fprintf(stderr, "Failed to open input file '%s': %s\n",
			argv[1], strerror(errno));
		return 1;
	}

start:
	ret = read(in_fd, &params, sizeof(params));
	if (ret == -1 && errno == EINTR)
		goto start;

	fprintf(stderr, "Read parameters buffer of size %d\n", ret);

	if (params.use.acc_bnr) {
		fprintf(stderr, "\n**** Bayer noise reduction parameters ****\n");
		displayBNR(&params);
	}
	if (params.use.acc_awb) {
		fprintf(stderr, "\n**** AWB parameters ****\n");
		displayGrid(&params.acc_param.awb.config.grid, "awb");
	}
	if (params.use.acc_awb_fr) {
		fprintf(stderr, "\n**** AWB filter response parameters ****\n");
		displayGrid(&params.acc_param.awb_fr.grid_cfg, "awb_fr");
	}

	if (params.use.acc_af) {
		fprintf(stderr, "\n**** AF parameters ****\n");
		displayGrid(&params.acc_param.af.grid_cfg, "af");
		displayAfFilter(&params);
	}

	if (params.use.lin_vmem_params) {
		fprintf(stderr, "\n**** Linearization parameters ****\n");
		displayLinLut(&params);
	}

	close(in_fd);

	return ret ? 1 : 0;
}
