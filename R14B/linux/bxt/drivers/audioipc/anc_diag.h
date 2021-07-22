#ifndef _ANC_DIAG_
#define _ANC_DIAG_

#define RESP_SUCCESS					0
#define RESP_FAILURE					1

struct cal_date {
	uint8_t year;
	uint8_t week;
	uint8_t pl;
};

struct cpd_ver {
	uint8_t s1;
	uint8_t s2;
	uint8_t s3;
};

struct anc_diag {
	uint8_t cal_resp;
	struct cal_date date;
	uint8_t ver_resp;
	struct cpd_ver ver;
	uint8_t id_resp;
	uint16_t cal_id;
	uint8_t sum_resp;
	uint32_t sum;
};

#endif

