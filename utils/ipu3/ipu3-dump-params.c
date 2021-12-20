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

	close(in_fd);

	return ret ? 1 : 0;
}
