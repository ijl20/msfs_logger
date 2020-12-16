//------------------------------------------------------------------------------
//						sim_logger
//  FSX IGC-standard logger
//  
//  Description:
//              reads the aircraft lat/long/alt and timestamp and writes an IGC-format log file
//
//              Written by Ian Forster-Lewis www.forsterlewis.com
//------------------------------------------------------------------------------

#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <strsafe.h>
#include <math.h>
#include <time.h>
#include <io.h>

#include "SimConnect.h"

// b21_logger version 
double version = 1.18;

// 'debug' parameters from command line that control various levels of diagnostic output
bool debug_info = false; // default 'debug' level - prints out startup, lift, errors
bool debug = false; // prints out lift factors as sim_probe runs
bool debug_calls = false; // greater level of debug - procedure calls
bool debug_events = false; // greater level of debug - events

bool menu_show_text = false; // boolean to decide whether to display debug text in FSX window

const int MAXBUF = 1000; // max length of an IGC file line or a filename
const int MAXC = 20; // max number of C records collectable from PLN file

char *igc_log_directory = "Modules\\sim_logger\\logs\\";

// full pathnames to each of the files
char flt_pathname[MAXBUF] = ""; 
char air_pathname[MAXBUF] = ""; 
char pln_pathname[MAXBUF] = ""; // we don't have to checksum the .PLN file
char wx_pathname[MAXBUF] = "";
char cmx_pathname[MAXBUF] = ""; // path to CumulusX config file
char cfg_pathname[MAXBUF] = ""; // path to aircraft.cfg config file - same folder as AIR
char xml_pathname[MAXBUF] = ""; // path to mission XML file - same folder as FLT

// shorter names for the files (folder/filename)
char flt_name[MAXBUF] = ""; 
char air_name[MAXBUF] = ""; 
char pln_name[MAXBUF] = "";
char wx_name[MAXBUF] = "";
char cmx_name[MAXBUF] = "";
char cfg_name[MAXBUF] = "";
char xml_name[MAXBUF] = "";

// length of checksum string
const int CHKSUM_CHARS = 6;

// printable checksum strings
char chksum[CHKSUM_CHARS+1]     = "000000";
char chksum_flt[CHKSUM_CHARS+1] = "000000";
char chksum_air[CHKSUM_CHARS+1] = "000000";
char chksum_wx[CHKSUM_CHARS+1]  = "000000";
char chksum_cmx[CHKSUM_CHARS+1] = "000000";
char chksum_cfg[CHKSUM_CHARS+1] = "000000"; // checksum for aircraft.cfg
char chksum_xml[CHKSUM_CHARS+1] = "000000"; // checksum for mission xml file
char chksum_all[CHKSUM_CHARS+1] = "000000"; // combined checksum for aircraft.cfg

// PLN data
int c_wp_count = 0; // count of C waypoints (#C records = this + 3)
char c_landing[MAXBUF];
char c[MAXC][MAXBUF];

// aircraft strings
char ATC_ID[MAXBUF];
char ATC_TYPE[MAXBUF];
char TITLE[MAXBUF];

// CumulusX code - set to non-zero if CX is locked
DWORD cx_code = 0;

// Weather code - set to non-zero if Wx is unchanged by user after WX file load
DWORD wx_code = 0;

// ThermalDescriptions.xml code - set to non-zero if file removed
DWORD therm_code = 0;

int     quit = 0;
HANDLE  hSimConnect = NULL;

static enum EVENT_ID {
    EVENT_SIM_START,
	EVENT_FLIGHT,
	EVENT_AIRCRAFT, // get event when aircraft loaded
	EVENT_FLIGHTPLAN,     // get event when flightplan loaded
	EVENT_WEATHER,        // get event when weather mode changed
	EVENT_MISSIONCOMPLETED,
// events for the FSX menu put up by logger under add-ons
    EVENT_MENU,
	EVENT_MENU_SHOW_TEXT,
	EVENT_MENU_HIDE_TEXT,
	EVENT_MENU_WRITE_LOG,
	EVENT_MENU_TEXT,
// the following are keystroke events useed in testing
    EVENT_Z,
    EVENT_X,
    EVENT_C,
    EVENT_V,
// Cx custom event
	EVENT_CX_CODE,
};

static enum DATA_REQUEST_ID {
    REQUEST_USER_POS,
	REQUEST_STARTUP_DATA,
	REQUEST_AIRCRAFT_DATA,
};

// GROUP_ID and INPUT_ID are used for keystroke events in testing
static enum GROUP_ID {
    GROUP_ZX,
	GROUP_MENU
};

static enum INPUT_ID {
    INPUT_ZX
};

static enum DEFINITION_ID {
//    DEFINITION_1,
    DEFINITION_USER_POS,
	DEFINITION_STARTUP,
	DEFINITION_AIRCRAFT,
};

struct UserStruct {
    double latitude;
    double longitude;
    double altitude; // meters
	INT32  sim_on_ground;
	INT32  zulu_time; // seconds
	INT32  rpm; // engine revs per min
};

// FSX FLIGHT DATA NEEDED FOR IGC LOG, e.g. startup time, aircraft info
struct StartupStruct {
	INT32 start_time;
	INT32 zulu_day;
	INT32 zulu_month;
	INT32 zulu_year;
};

struct AircraftStruct { // variable length strings ATC ID, ATC TYPE, TITLE
	char strings[1];
};

//*******************************************************************************

// create var to hold user plane position
UserStruct user_pos;

// startup_data holds the data picked up from FSX at the start of each flight
StartupStruct startup_data;

//*******************************************************************************
//********************** CHECKSUM CALCULATION ***********************************
//*******************************************************************************

// return codes from chksum_igc_file()
static enum CHKSUM_RESULT {
    CHKSUM_OK,
	CHKSUM_NOT_FOUND,
	CHKSUM_TOO_SHORT,
	CHKSUM_BAD,
	CHKSUM_FILE_ERROR,
};

// number of characters to include in checksum
const int CHK_CHARS = 63; // number of chars in chk_source and chk_map
// list of CHK_CHARS characters to be mapped into checksum (other chars ignored)
char *chk_source = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.abcdefghijklmnopqrstuvwxyz";
// map table for char->int 0..(CHK_CHARS-1)
int chk_map[CHK_CHARS]    = { 14,46,51,8,26,2,32,39,29,
							 37,4,44,20,61,22,58,16,25,
							 60,13,31,53,11,50,6,38,41,
							 23,56,17,1,19,45,10,28,15,
							 36,9,57,12,49,33,3,24,30,
							 62,47,5,43,0,27,52,34,55,
							 21,54,59,18,48,35,40,7,42};

// modulo value for chksum_index which increments through the file
const int CHKSUM_MAX_INDEX = 1987;

struct ChksumData {
	int index;
	int num[CHKSUM_CHARS];
};

// incrementally update checksum given current char c
void incr_chksum(ChksumData *chk_data, char c) {
	unsigned int c_pos = 0;
	// convert c to int via chk_source array
	while (c_pos<CHK_CHARS && chk_source[c_pos]!=c) c_pos++;
	// if c not found then simply return (only need checksum valid chars)
	if (c_pos==CHK_CHARS) return;

	// now c_pos is index of c in char_source, get mapped number
	int map_num = chk_map[(c_pos + chk_data->index) % CHK_CHARS];
	for (int i=0; i<CHKSUM_CHARS; i++) {
		chk_data->num[i] = chk_map[(chk_data->num[i]+map_num+i) % CHK_CHARS];
	}
	// Increment checksum_index
	chk_data->index = (chk_data->index + 1) % CHKSUM_MAX_INDEX;
}

// update chksum_num based on input string s
void chksum_string(ChksumData *chk_data, char *s) {
	for (unsigned int i=0; i<strlen(s); i++) 
		incr_chksum(chk_data, s[i]);
}

// update chksum_num based on BINARY input string s
void chksum_binary(ChksumData *chk_data, char *s, int count) {
	for (int i=0; i<count; i++) 
		incr_chksum(chk_data, s[i]);
}

// convert chk_data.num[] into string chksum
void chksum_to_string(char chksum[CHKSUM_CHARS+1], ChksumData chk_data) {
	for (int i=0; i<CHKSUM_CHARS;i++) 
		chksum[i] = chk_source[chk_data.num[i] % 36];
}

void chksum_reset(ChksumData *chk_data) {
	chk_data->index = 1;
	for (int i=0; i<CHKSUM_CHARS;i++) chk_data->num[i]=i;
}

CHKSUM_RESULT chksum_binary_file(char chksum[CHKSUM_CHARS+1], char *filepath) {
	FILE *f;
	errno_t err;
	char buf[MAXBUF];
	int read_count; // number of chars read
	// calculated checksum as sequence of ints 0..CHK_CHARS
	ChksumData chk_data;
	
	chksum_reset(&chk_data);

	if( (err = fopen_s(&f, filepath, "rb")) != 0 ) {
        strcpy_s(chksum, CHKSUM_CHARS+1, "000000");
		return CHKSUM_FILE_ERROR;
	}
	while (!feof(f)) {
		read_count = fread(buf, sizeof(char),sizeof(buf),f);
		chksum_binary(&chk_data, buf, read_count);
	}
	chksum_to_string(chksum, chk_data);
	fclose(f);
	return CHKSUM_OK;
}

// starts_bracket returns -1 if string doesn't have '[' as first non-space char
// otherwise it returns the index of the '[' char
int starts_bracket(char *s) {
    unsigned int i = 0;
    //debug
    //printf("starts_bracket %s\n",s);
    while (i<10 && i<strlen(s)) {
        if (s[i]=='[') return i;        // found '[' => return
        else if (s[i]!=' ') return -1;  // string starts with non-'[' 
        i++;
    }
    return -1;                      
}

bool perf_match(char *line_buf) {
    const int PERF_COUNT = 11;
    char *strings[PERF_COUNT];
    int length[PERF_COUNT]; // number of chars into string to detect
    int pos;
    int i = 0;

    strings[0]="[airplane_geometry]";
    length[0] = 5;
    strings[1]="[flaps.";
    length[1] = 4;
    strings[2]="[flight_tuning]";
    length[2] = 4;
    strings[3]="[water_ballast_system]";
    length[3] = 3;
    strings[4]="[weight_and_balance]";
    length[4] = 3;
    strings[5]="[generalenginedata]";
    length[5] = 11;
    strings[6]="[jet_engine]";
    length[6] = 4;
    strings[7]="[piston_engine]";
    length[7] = 4;
    strings[8]="[propeller]";
    length[8] = 4;
    strings[9]="[turbineenginedata]";
    length[9] = 6;
    strings[10]="[turboprop_engine]";
    length[10] = 6;

    pos = starts_bracket(line_buf);
    //debug
    //printf(" (pos=%+d) ",pos);
    if (pos<0) return false;
    while (i<PERF_COUNT) {
        if ( strncmp(line_buf+pos,strings[i],length[i]) == 0 ) {
            //debug
            //printf("PERF_MATCH %s\n", strings[i]);
            return true;
        }
        i++;
    }
    return false;
}

// chksum_cfg_file calculates a checksum for aircraft.cfg file only including sections
// of the file that affect performance
CHKSUM_RESULT chksum_cfg_file(char chksum[CHKSUM_CHARS+1], char *filepath) {
	FILE *f;
	errno_t err;
	char line_buf[MAXBUF];
    bool in_perf_section = false;

	// calculated checksum as sequence of ints 0..CHK_CHARS
	ChksumData chk_data;
	
	chksum_reset(&chk_data);

	if( (err = fopen_s(&f, filepath, "r")) != 0 ) {
        strcpy_s(chksum, CHKSUM_CHARS+1, "000000");
		return CHKSUM_FILE_ERROR;
	}
    while (fgets(line_buf, MAXBUF, f)!=NULL) {
        //debug
        //if (in_perf_section) printf("    PERF ");
        //else printf("NON-PERF ");
        //printf(">%s",line_buf);
		if (!in_perf_section) in_perf_section = perf_match(line_buf);
        else {
            if (starts_bracket(line_buf)<0)
                chksum_string(&chk_data, line_buf);
            else in_perf_section = perf_match(line_buf);
        }
    }
	chksum_to_string(chksum, chk_data);
	fclose(f);
	return CHKSUM_OK;
}

// This routine is used to *check* the checksum at the end of an IGC file
// the checksum will be stored in the final 'G' record.
// Only alphanumeric characters before the 'G' record contribute to the checksum.
CHKSUM_RESULT chksum_igc_file(char chksum[CHKSUM_CHARS+1], char *filepath) {
	FILE *f;
	errno_t err;
	char line_buf[MAXBUF];
	// calculated checksum as sequence of ints 0..CHK_CHARS
	ChksumData chk_data;
	
	chksum_reset(&chk_data);

	if( (err = fopen_s(&f, filepath, "r")) != 0 ) {
		return CHKSUM_FILE_ERROR;
	}
	while (fgets(line_buf, MAXBUF, f)!=NULL) {
		if (line_buf[0]=='G') break;
		if (strncmp(line_buf,"L FSX GENERAL", 13)==0) printf("%s",line_buf+6);
		chksum_string(&chk_data, line_buf);
	}
	if (line_buf[0]!='G') {
			return CHKSUM_NOT_FOUND;
	}

	if (strlen(line_buf)<CHKSUM_CHARS+1) {
			return CHKSUM_TOO_SHORT;
	}
	chksum_to_string(chksum, chk_data);
	for (int i=0; i<CHKSUM_CHARS; i++) {
		if (chksum[i]!=line_buf[i+1]) {
			return CHKSUM_BAD;
		}
	}
	fclose(f);
	return CHKSUM_OK;
}

CHKSUM_RESULT check_file(char *pfilepath) {
	char chksum[CHKSUM_CHARS+1] = "000000";
	return chksum_igc_file(chksum, pfilepath);
}

// this routine produces a general checksum for the
// FLT, WX, CMX, AIR, aircraft.cfg files
// so if this is correct the user does not have to look at the 
// individual checksums
CHKSUM_RESULT chksum_chksum(char chksum[CHKSUM_CHARS+1]) {
	ChksumData chk_data;

	chksum_reset(&chk_data);

	chksum_string(&chk_data,chksum_flt);
	chksum_string(&chk_data,chksum_air);
	chksum_string(&chk_data,chksum_wx);
	chksum_string(&chk_data,chksum_cmx);
	chksum_string(&chk_data,chksum_cfg);
	chksum_string(&chk_data,chksum_xml);
	if (cx_code==0) 
		chksum_string(&chk_data, "CX UNLOCKED");
	else
		chksum_string(&chk_data, "CX LOCKED");

	if (wx_code==0) 
		chksum_string(&chk_data, "WX UNLOCKED");
	else
		chksum_string(&chk_data, "WX LOCKED");

    if (therm_code==0)
        chksum_string(&chk_data, "THERM FILE PRESENT");
    else
        chksum_string(&chk_data, "NO THERM FILE");

	chksum_to_string(chksum, chk_data);
	return CHKSUM_OK;
}

//*******************************************************************
//*******************************************************************
//******************    PARSE THE PLN FILE **************************
//*******************************************************************

// utility function - copy 'n' chars from src to dest
void cpy(char *dest, int max, char *src, int n) {
    if (n>max) return;
    for (int i=0;i<n;i++) dest[i]=src[i];
    return;
}

// this very useful function converts in(UNICODE) to s(ascii) and strips odd chars
void clean_string(char *s, wchar_t *in) {
	unsigned int i = 0;
    const wchar_t *PLN_WCHARS = L"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz.<>, ";
	while (i<wcslen(in)) {
        if (wcschr(PLN_WCHARS,in[i])==NULL) 
			s[i] = ' ';
		else
			s[i] = char(in[i]);
		i++;
    }
	s[i] = '\0';
}

CHKSUM_RESULT pln_to_c(char *filepath) {
	FILE *f;
	errno_t err;

	char line_buf[MAXBUF];
    char s[MAXBUF]; // general buffer
	wchar_t in_buf[MAXCHAR*4]; // line input buffer

    char *ptr_value; // temporary pointer into read line from PLN
    char *ptr_end; // temp pointer to end of value
    int value_length; // length of value up to following '<' char

    char lat_NS = ' ';
    int lat_degs = 0;
    int lat_mins = 0;
    float lat_secs = 0;
    char long_NS = ' ';
    int long_degs = 0;
    int long_mins = 0;
    float long_secs = 0;
    char comma = ','; // temp placeholder for sscanf_s

    //debug
    //printf("parsing %s\n",filepath);

    c_wp_count = 0;
    strcpy_s(c[0],MAXBUF,"");
    strcpy_s(c[1],MAXBUF,"");
    strcpy_s(c_landing,MAXBUF,"");

	if( (err = fopen_s(&f, filepath, "r, ccs=UNICODE")) != 0 ) {
		return CHKSUM_FILE_ERROR;
	}
	// use current PC time for declaration date in C header record
	__time64_t ltime;
	struct tm today;
    _time64(&ltime);
    _localtime64_s( &today, &ltime );
	//strcpy_s(fn, MAXBUF, "\"");
	//strcat_s(fn, igc_log_directory);
	strftime(s, MAXBUF, "C%d%m%y%H%M%S000000000100NO TASK", &today );
	strcpy_s(c[0],MAXBUF,s);
    //debug
    //c[0][32]='\0';
    //printf("first C record: %s\n",c[0]);

    while (fgetws(in_buf, MAXBUF, f)!=NULL) {
		//debug
		//wprintf_s(L"- %s",in_buf);
        clean_string(line_buf, in_buf);
        //debug
        //printf("c %s\n",line_buf);
		//continue;
        // now test for each record we want from PLN

        ptr_value = strstr(line_buf,"<Title>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-7);
                strncpy_s(s,MAXBUF,ptr_value+7,value_length);
                strcpy_s(c[0]+25,MAXBUF-25,s);
				strcat_s(c[0]+25,MAXBUF,"\n");
                //debug
                //printf("Title \"%s\"\n", c[0]+25);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DepartureName>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-15);
                strncpy_s(s,MAXBUF,ptr_value+15,value_length);
                strcpy_s(c[1]+18,MAXBUF-18,s);
				strcat_s(c[1]+18,MAXBUF,"\n");
                //debug
                //printf("DepartureName \"%s\"\n", c[1]+18);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DestinationName>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-17);
                strncpy_s(s,MAXBUF,ptr_value+17,value_length);
                strcpy_s(c_landing+18,MAXBUF-18,s);
				strcat_s(c_landing+18,MAXBUF,"\n");
                //debug
                //printf("DestinationName \"%s\"\n", c_landing+18);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DepartureLLA>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                sscanf_s(ptr_value+14, "%c %d %d %f %c %c %d %d %f",
                    &lat_NS,1, &lat_degs, &lat_mins, &lat_secs, &comma, 1,
                    &long_NS,1, &long_degs, &long_mins, &long_secs);

                lat_secs = lat_secs / 60 * 1000;
                long_secs = long_secs / 60 * 1000;
                sprintf_s(s,MAXBUF,"C%02.2d%02.2d%03.0f%c%03.3d%02.2d%03.0f%c",
                    lat_degs, lat_mins, lat_secs, lat_NS, 
                    long_degs, long_mins, long_secs, long_NS );
                cpy(c[1],MAXBUF,s,18);
                //debug                C  DD    MM   MMM   N   DDD   MM    MMM  E
                //printf("DepartureLLA \"C %02.2d %02.2d %03.0f %c %03.3d %02.2d %03.0f %c\"\n",
                //    lat_degs, lat_mins, lat_secs, lat_NS, 
                //    long_degs, long_mins, long_secs, long_NS );
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<DestinationLLA>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                sscanf_s(ptr_value+16, "%c %d %d %f %c %c %d %d %f",
                    &lat_NS,1, &lat_degs, &lat_mins, &lat_secs, &comma, 1,
                    &long_NS,1, &long_degs, &long_mins, &long_secs);

                lat_secs = lat_secs / 60 * 1000;
                long_secs = long_secs / 60 * 1000;
                sprintf_s(s,MAXBUF,"C%02.2d%02.2d%03.0f%c%03.3d%02.2d%03.0f%c",
                    lat_degs, lat_mins, lat_secs, lat_NS, 
                    long_degs, long_mins, long_secs, long_NS );
                cpy(c_landing,MAXBUF,s,18);
                //debug                C  DD    MM   MMM   N   DDD   MM    MMM  E
                //printf("DestinationLLA \"C %02.2d %02.2d %03.0f %c %03.3d %02.2d %03.0f %c\"\n",
                //    lat_degs, lat_mins, lat_secs, lat_NS, 
                //    long_degs, long_mins, long_secs, long_NS );
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<ATCWaypoint ");
        if (ptr_value!=NULL) {
            c_wp_count++;
            ptr_end = strchr(ptr_value, '>');
            if (ptr_end!=NULL) {
                value_length = (int)(ptr_end-ptr_value-18);
                strncpy_s(s,MAXBUF,ptr_value+17,value_length);
                strcpy_s(c[c_wp_count+1]+18,MAXBUF-18,s);
				strcat_s(c[c_wp_count+1]+18,MAXBUF,"\n");
                //debug
                //printf("Waypoint \"%s\"\n", c[c_wp_count+1]+18);
            }
            continue;
        }
        ptr_value = strstr(line_buf,"<WorldPosition>");
        if (ptr_value!=NULL) {
            ptr_end = strchr(ptr_value+1, '<');
            if (ptr_end!=NULL) {
                sscanf_s(ptr_value+15, "%c %d %d %f %c %c %d %d %f",
                    &lat_NS,1, &lat_degs, &lat_mins, &lat_secs, &comma, 1,
                    &long_NS,1, &long_degs, &long_mins, &long_secs);

                lat_secs = lat_secs / 60 * 1000;
                long_secs = long_secs / 60 * 1000;
                sprintf_s(s,MAXBUF,"C%02.2d%02.2d%03.0f%c%03.3d%02.2d%03.0f%c",
                    lat_degs, lat_mins, lat_secs, lat_NS, 
                    long_degs, long_mins, long_secs, long_NS );
                cpy(c[c_wp_count+1],MAXBUF,s,18);
                //debug                C  DD    MM   MMM   N   DDD   MM    MMM  E
                //printf("WorldPosition \"C %02.2d %02.2d %03.0f %c %03.3d %02.2d %03.0f %c\"\n",
                //    lat_degs, lat_mins, lat_secs, lat_NS, 
                //    long_degs, long_mins, long_secs, long_NS );
            }
            continue;
        }
    }
	fclose(f);
	//debug
	//printf("\nParsing PLN completed.\n");
	// inject turnpoint count (waypoints - 2) into c[0] record
	if (c_wp_count>2) {
		sprintf_s(s, MAXBUF,"%02.2d",c_wp_count-2);
		cpy(c[0]+23,MAXBUF-23,s,2);
	} else cpy(c[0]+23,MAXBUF-23,"00",2);
	//return CHKSUM_OK;
    //debug
	if (debug) {
		printf("first C record: %s",c[0]);
		printf("departure:      %s",c[1]);
		for (int i=0; i<c_wp_count; i++) printf("WP:             %s",c[i+2]);
		printf("landing:        %s",c_landing);
	}
	return CHKSUM_OK;

}


//*******************************************************************
//**************** find short filename      *************************

void path_to_name(char name[MAXBUF], char path[MAXBUF]) {
	int i;
	bool found1 = false; // flag to say we've found 1st '\' char
	bool found2 = false;

	// first check if file is there
    if(_access_s(path, 0) != 0) {
        strcpy_s(name, MAXBUF,"not found");
        return;
	}

	// ok we've found the file, so lets abbreviate the name
	i = strlen(path);
	while (i>0 && !found2) {
		if (path[--i]=='\\') { 
			if (!found1) found1 = true;
			else found2 = true;
		}
	}	
	strcpy_s(name, MAXBUF,path+i+1);
}

//*******************************************************************
//*****************  SIMCONNECT             *************************
//*******************************************************************
//*******************************************************************

//*******************************************************************
// igc file logger vars
//*******************************************************************
const int IGC_TICK_COUNT = 4; // log every 4 seconds
const INT32 IGC_MAX_RECORDS = 40000; // log a maximum of this many 'B' records.
const INT32 IGC_MIN_RECORDS = 4; // don't record an IGC file if it is short
const INT32 IGC_MIN_FLIGHT_SECS_TO_LANDING = 80; // don't trigger a log save on landing unless
                                                 // airborne for at least 80 seconds

int igc_tick_counter = 0; // variable to keep track of how many ticks we've counted 0..IGC_TICK_COUNT
INT32 igc_record_count = 0; // count of how many 'B' records we've recorded

INT32 igc_takeoff_time; // note time of last "SIM ON GROUND"->!(SIM ON GROUND) transition
INT32 igc_prev_on_ground = 0;

// struct of data in an IGC 'B' record
struct igc_b {
	INT32 zulu_time;
	double latitude;
	double longitude;
	double altitude;
    double rpm;
};

// array to hold all the 'B' records
igc_b igc_pos[IGC_MAX_RECORDS];

//**********************************************************************************
//******* IGC FILE ROUTINES                                                 ********
//**********************************************************************************
//**********************************************************************************


void igc_reset_log() {
	//c_wp_count = 0;
	igc_record_count = 0;
}

void get_aircraft_data() {
    HRESULT hr;
    // set data request
    hr = SimConnect_RequestDataOnSimObject(hSimConnect, 
                                            REQUEST_AIRCRAFT_DATA, 
                                            DEFINITION_AIRCRAFT, 
                                            SIMCONNECT_OBJECT_ID_USER,
                                            SIMCONNECT_PERIOD_ONCE); 

}

// this routine will be called each time:
//  * sim start
//  * change of aircraft
//  * change of Wx

void get_startup_data() {
    HRESULT hr;
    // set data request
    hr = SimConnect_RequestDataOnSimObject(hSimConnect, 
                                            REQUEST_STARTUP_DATA, 
                                            DEFINITION_STARTUP, 
                                            SIMCONNECT_OBJECT_ID_USER,
                                            SIMCONNECT_PERIOD_ONCE); 

	// now get aircraft data
	get_aircraft_data();
}

void get_user_pos_updates() {
    HRESULT hr;
    if (debug_calls) printf(" ..entering get_user_pos_updates()..");
	// set data request
	hr = SimConnect_RequestDataOnSimObject(hSimConnect, 
											REQUEST_USER_POS, 
											DEFINITION_USER_POS, 
											SIMCONNECT_OBJECT_ID_USER,
											SIMCONNECT_PERIOD_SECOND); 
    if (debug_calls) printf(" ..leaving get_user_pos_updates()..\n");
}

void igc_log_point(UserStruct p) {
	if (igc_record_count<IGC_MAX_RECORDS) {
		if (igc_record_count==0 || p.zulu_time!=igc_pos[igc_record_count-1].zulu_time) {
			igc_pos[igc_record_count].latitude = p.latitude;
			igc_pos[igc_record_count].longitude = p.longitude;
			igc_pos[igc_record_count].altitude = p.altitude;
			igc_pos[igc_record_count].zulu_time =p.zulu_time;
			igc_pos[igc_record_count].rpm =p.rpm;
			igc_record_count++;
		}
	}
}

void igc_write_file(char *reason) {
	FILE *f;
	char buf[MAXBUF];
	char s[MAXBUF]; // buffer to how igc records before writing to file
	char fn[MAXBUF];
	errno_t err;
	ChksumData chk_data;
	char chksum[CHKSUM_CHARS+1] = "000000";

	if (debug) {
		printf("flt_pathname=%s\n", flt_pathname);
		printf("chksum_flt=%s\n\n", chksum_flt);
		printf("air_pathname=%s\n", air_pathname);
		printf("chksum_air=%s\n\n", chksum_air);
		printf("pln_pathname=%s (no checksum)\n\n", pln_pathname);
		printf("wx_pathname=%s\n", wx_pathname);
		printf("chksum_wx=%s\n\n", chksum_wx);
		printf("cmx_pathname=%s\n", cmx_pathname);
		printf("chksum_cmx=%s\n\n", chksum_cmx);
		printf("cfg_pathname=%s\n", cfg_pathname);
		printf("chksum_cfg=%s\n\n", chksum_cfg);
	}

	path_to_name(flt_name, flt_pathname);
	path_to_name(air_name, air_pathname);
	path_to_name(pln_name, pln_pathname);
	path_to_name(wx_name, wx_pathname);
	path_to_name(cmx_name, cmx_pathname);
	path_to_name(cfg_name, cfg_pathname);
	path_to_name(xml_name, xml_pathname);

    // check for existence of ThermalDescriptions.xml and set therm_code=0 if so
    if(_access_s("ThermalDescriptions.xml", 0) != 0) {
        therm_code = 1;
    } else {
        therm_code = 0;
    }
    
	char *flight_fn1 = strrchr(flt_pathname, '\\');
	if (flight_fn1==NULL) flight_fn1 = flt_pathname;
	else flight_fn1++;
	char *dot = strrchr(flight_fn1,'.');
	char flight_filename[1000];
	if (dot!=NULL) {
		//strncpy(flight_fn2, flight_fn1, dot-flight_fn1+1);
		errno_t err = strncpy_s( flight_filename, _countof(flight_filename), flight_fn1, dot-flight_fn1);
	} else {
		errno_t err = strcpy_s( flight_filename, flight_fn1);
	}

	// make the log filename in fn - file will go in logger.exe folder
	__time64_t ltime;
	struct tm today;
    _time64(&ltime);
    _localtime64_s( &today, &ltime );
	//strcpy_s(fn, MAXBUF, "\"");
	//strcat_s(fn, igc_log_directory);
	strcpy_s(fn, MAXBUF, igc_log_directory);
	strcat_s(fn, ATC_ID);
	strcat_s(fn, "_");
	strcat_s(fn, flight_filename);
	strftime(buf, MAXBUF, "_%Y-%m-%d_%H%M", &today );
	strcat_s(fn, buf);
	if (strlen(reason)>1) {
		strcat_s(fn, MAXBUF, "(");
		strcat_s(fn, MAXBUF, reason);
		strcat_s(fn, MAXBUF, ")");
	}
	strcat_s(fn, MAXBUF, ".igc");
	//strcat_s(fn, "\"");

	// debug
	if (debug) printf("\nWriting IGC file: %s\n",fn);

	if( (err = fopen_s(&f, fn, "w")) != 0 ) {
		char error_text[200];
		
		sprintf_s(error_text, 
				sizeof(error_text), 
				"igc_logger v%.2f could not write log to file \"%s\"", 
				version, 
				fn);

		HRESULT hr = SimConnect_Text(hSimConnect, 
									SIMCONNECT_TEXT_TYPE_SCROLL_RED, 
									15.0, 
									EVENT_MENU_TEXT,
									sizeof(error_text), 
									error_text);
		return;
	} else {
		chksum_reset(&chk_data);
		// ok we've opened the log file - lets write all the data to it
		sprintf_s(s,MAXBUF,         "AXXX sim_logger v%.2f\n", version); // manufacturer
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,		   "HFDTE%02.2d%02.2d%02.2d\n", startup_data.zulu_day,     // date
														startup_data.zulu_month,
														startup_data.zulu_year % 1000);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFFXA035\n");                        // gps accuracy
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFPLTPILOTINCHARGE: not recorded\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFCM2CREW2: not recorded\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFGTYGLIDERTYPE:%s\n", TITLE);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFGIDGLIDERID:%s\n", ATC_ID);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFDTM100GPSDATUM: WGS-1984\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFRFWFIRMWAREVERSION: %.2f\n", version);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFRHWHARDWAREVERSION: 2009\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFFTYFRTYPE: sim_logger by Ian Forster-Lewis\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFGPSGPS:Microsoft Flight Simulator\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFPRSPRESSALTSENSOR: Microsoft Flight Simulator\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFCIDCOMPETITIONID:%s\n", ATC_ID);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,         "HFCCLCOMPETITIONCLASS:Microsoft Flight Simulator\n");
		chksum_string(&chk_data, s); fprintf(f, s);

							// extension record to say info at end of 'B' recs
							// FXA = fix accuracy
							// SIU = satellites in use
							// ENL = engine noise level 000-999
		sprintf_s(s,MAXBUF,         "I023638FXA3941ENL\n"); 
		chksum_string(&chk_data, s); fprintf(f, s);

		// Task (C) records
		if (c_wp_count>1) {
			chksum_string(&chk_data, c[0]); fprintf(f, c[0]);
			chksum_string(&chk_data, c[1]); fprintf(f, c[1]);
			for (int i=0; i<c_wp_count; i++) {
				chksum_string(&chk_data, c[i+2]); fprintf(f, c[i+2]);
			}
			chksum_string(&chk_data, c_landing); fprintf(f, c_landing);
		}

		// FSX Comment (L) records
		strftime( buf, 50, "L FSX date/time on users PC:  %Y-%m-%d %H:%M\n", &today );            // date
		sprintf_s(s,MAXBUF,buf);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX FLT filename %s\n", flt_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX FLT checksum            %s (%s)\n", chksum_flt, flt_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX PLN filename %s\n", pln_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		//sprintf_s(s,MAXBUF,		   "L FSX PLN checksum %s\n", chksum_pln);
		//chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX WX filename %s\n", wx_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX WX checksum             %s (%s)\n", chksum_wx, wx_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX CMX filename %s\n", cmx_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX CMX checksum            %s (%s)\n", chksum_cmx, cmx_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,		   "L FSX mission checksum        %s (%s)\n", chksum_xml, xml_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		sprintf_s(s,MAXBUF,		   "L FSX aircraft.cfg checksum   %s (%s)\n", chksum_cfg, cfg_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		//sprintf_s(s,MAXBUF,		   "L FSX AIR filename %s\n", air_pathname);
		//chksum_string(&chk_data, s); fprintf(f, s);
		sprintf_s(s,MAXBUF,		   "L FSX AIR checksum            %s (%s)\n", chksum_air, air_name);
		chksum_string(&chk_data, s); fprintf(f, s);

		// write CumulusX status locked/unlocked
		if (cx_code==0)
			sprintf_s(s,MAXBUF,		   "L FSX CumulusX status:        UNLOCKED\n");
		else
			sprintf_s(s,MAXBUF,		   "L FSX CumulusX status:        LOCKED OK\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		// write wx status (unlocked if user has entered weather menu
		// after a WX file load
		if (wx_code==0)
			sprintf_s(s,MAXBUF,		   "L FSX WX status=              UNLOCKED\n");
		else
			sprintf_s(s,MAXBUF,		   "L FSX WX status=              LOCKED OK\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		// ThermalDescriptions.xml entry
		if (therm_code==0)
			sprintf_s(s,MAXBUF,		   "L FSX ThermalDescriptions.xml STILL BEING USED\n");
		else
			sprintf_s(s,MAXBUF,		   "L FSX ThermalDescriptions.xml REMOVED OK\n");
		chksum_string(&chk_data, s); fprintf(f, s);

		// now calculate a value for the GENERAL CHECKSUM
		chksum_chksum(chksum_all);
		sprintf_s(s,MAXBUF,		   "L FSX GENERAL CHECKSUM            %s  <---- CHECK THIS FIRST\n", chksum_all);
		chksum_string(&chk_data, s); fprintf(f, s);

		// now do the 'B' location records
		for (INT32 i=0; i<igc_record_count; i++) {
			int hours = igc_pos[i].zulu_time / 3600;
			int minutes = (igc_pos[i].zulu_time - hours * 3600 ) / 60;
			int secs = igc_pos[i].zulu_time % 60;
			char NS = (igc_pos[i].latitude>0.0) ? 'N' : 'S';
			char EW = (igc_pos[i].longitude>0.0) ? 'E' : 'W';
			double abs_latitude = fabs(igc_pos[i].latitude);
			double abs_longitude = fabs(igc_pos[i].longitude);
			int lat_DD = int(abs_latitude);
			int lat_MM = int( (abs_latitude - float(lat_DD)) * 60.0);
			int lat_mmm = int( (abs_latitude - float(lat_DD) - (float(lat_MM) / 60.0)) * 60000.0);
			int long_DDD = int(abs_longitude);
			int long_MM = int((abs_longitude - float(long_DDD)) * 60.0);
			int long_mmm = int((abs_longitude - float(long_DDD) - (float(long_MM) / 60.0)) * 60000.0);
			int altitude = int(igc_pos[i].altitude);
            int FXA = 27;
            int ENL = (int(igc_pos[i].rpm)>9990)? 999 : int(igc_pos[i].rpm) / 10;

//			sprintf_s(s,MAXBUF,     "B %02.2d %02.2d %02.2d %02.2d %02.2d %03.3d %c %03.3d %02.2d %03.3d %c A %05.5d %05.5d 000\n",
			sprintf_s(s,MAXBUF,     "B%02.2d%02.2d%02.2d%02.2d%02.2d%03.3d%c%03.3d%02.2d%03.3d%cA%05.5d%05.5d%03.3d%03.3d\n",
				    hours, minutes, secs,
					lat_DD, lat_MM, lat_mmm, NS,
					long_DDD, long_MM, long_mmm, EW,
					altitude, altitude, FXA, ENL);
			chksum_string(&chk_data, s); fprintf(f, s);
		}
		chksum_to_string(chksum, chk_data);
		fprintf(f,         "G%s\n",chksum);

		fclose(f);

		char file_write_text[132];
		
		sprintf_s(file_write_text, 
				sizeof(file_write_text), 
				"igc_logger v%.2f wrote %s", 
				version, 
				fn);

		HRESULT hr = SimConnect_Text(hSimConnect, 
									SIMCONNECT_TEXT_TYPE_PRINT_GREEN, 
									6.0, 
									EVENT_MENU_TEXT, //sizeof("TESTING"), "TESTING"); 
									sizeof(file_write_text), 
									file_write_text);
	}
}

// this routine used to trigger an igc_file_write, but not used now
void igc_ground_check(INT32 on_ground, INT32 zulu_time) {
	// test for start of flight
	if (igc_record_count<2) {
		// if at start of flight then set up initial 'on ground' status
		igc_prev_on_ground = on_ground;
	} else
	// test for takeoff
	if (igc_prev_on_ground != 0 && on_ground == 0) {
		igc_prev_on_ground = 0; // remember current state is NOT on ground
		igc_takeoff_time = zulu_time; // record current time
		if (debug) printf("\nTakeoff detected\n"); 
	} else 
	// test for landing		
	if (igc_prev_on_ground == 0 && on_ground != 0 && // just landed
	       (zulu_time - igc_takeoff_time)>IGC_MIN_FLIGHT_SECS_TO_LANDING) { 
			   // AND was airborn long enough
  		if (debug) printf("\nLanding detected\n"); 
		igc_prev_on_ground = 1;
	} else {
		igc_prev_on_ground = on_ground;
	}
}

//*********************************************************************************************
//********** this is the main message handling loop of logger, receiving messages from FS **
//*********************************************************************************************
void CALLBACK MyDispatchProcSO(SIMCONNECT_RECV* pData, DWORD cbData, void *pContext)
{   
    //HRESULT hr;
    //printf("\nIn dispatch proc");
	char *c_pointer; // pointer to last '.' in FLT pathname

    switch(pData->dwID)
    {
        case SIMCONNECT_RECV_ID_EVENT:
        {
            SIMCONNECT_RECV_EVENT *evt = (SIMCONNECT_RECV_EVENT*)pData;

            switch(evt->uEventID)
            {
				case EVENT_MENU_SHOW_TEXT:
					if (debug) printf(" [EVENT_MENU_SHOW_TEXT] ");
					menu_show_text = true;
                    break;
					
				case EVENT_MENU_HIDE_TEXT:
					if (debug) printf(" [EVENT_MENU_HIDE_TEXT] ");
					menu_show_text = false;
                    break;
					
				case EVENT_MENU_WRITE_LOG:
					if (debug) printf(" [EVENT_MENU_WRITE_LOG]\n");
					igc_write_file("");
                    break;
					
                case EVENT_SIM_START:
					if (debug) printf(" [EVENT_SIM_START]\n");
                    // Sim has started so turn the input events on
                    //hr = SimConnect_SetInputGroupState(hSimConnect, INPUT_ZX, SIMCONNECT_STATE_ON);

					// get startup data e.g. zulu time, aircraft info
					get_startup_data();
                    break;

                case EVENT_MISSIONCOMPLETED:
					if (debug) printf(" [EVENT_MISSIONCOMPLETED]\n");
					// always write an IGC file on mission completion
					// igc_write_file("");
                    break;

                case EVENT_MENU_TEXT:
					if (debug_events) printf(" [EVENT_MENU_TEXT] ");
                    break;

                case EVENT_Z: // keystroke Z
                    break;

                case EVENT_X: // keystroke X
                    break;
                
				case EVENT_CX_CODE: // CumulusX reporting a UI unlock
					if (debug) printf(" [EVENT_CX_CODE]=%d\n",evt->dwData);
					cx_code = evt->dwData;
					break;

                default:
                    if (debug) printf("\nUnknown event: %d\n", evt->uEventID);
                    break;
            }
            break;
        }

        case SIMCONNECT_RECV_ID_EVENT_WEATHER_MODE:
        {
            SIMCONNECT_RECV_EVENT *evt = (SIMCONNECT_RECV_EVENT*)pData;

            switch(evt->uEventID)
            {
				case EVENT_WEATHER: // User has changed weather
					if (debug) printf(" [EVENT_WEATHER]\n");
					wx_code = 0;
					break;

				default:
                    if (debug) printf("\nUnknown weather mode event: %d\n", evt->uEventID);
                    break;
			}
			break;
		}
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
        {
            SIMCONNECT_RECV_SIMOBJECT_DATA *pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*) pData;

            switch(pObjData->dwRequestID)
            {

                case REQUEST_STARTUP_DATA:
                {
					// startup data will be requested at SIM START
					if (debug) printf(" [REQUEST_STARTUP_DATA] ");
                    StartupStruct *pU = (StartupStruct*)&pObjData->dwData;
					startup_data.start_time = pU->start_time;
					startup_data.zulu_day = pU->zulu_day;
					startup_data.zulu_month = pU->zulu_month;
					startup_data.zulu_year = pU->zulu_year;
					if (debug) printf("\nStartup data: Zulu time=%d-%d-%d@%d\n",
									  startup_data.zulu_year, startup_data.zulu_month,
									  startup_data.zulu_day, startup_data.start_time);
					// now we have the startup data, request user pos updates
					get_user_pos_updates();
                    break;
                }

                case REQUEST_AIRCRAFT_DATA:
                {
					// startup data will be requested at SIM START
					if (debug) printf(" [REQUEST_AIRCRAFT_DATA] ");
					AircraftStruct *pS = (AircraftStruct*)&pObjData->dwData;
                    char *pszATC_ID;
                    char *pszATC_TYPE;
                    char *pszTITLE;
                    DWORD cbATC_ID;
					DWORD cbATC_TYPE;
					DWORD cbTITLE;

					// Note how the third parameter is moved along the data received
                    if(SUCCEEDED(SimConnect_RetrieveString(pData, cbData, &pS->strings, &pszATC_ID, &cbATC_ID)) &&
                       SUCCEEDED(SimConnect_RetrieveString(pData, cbData, pszATC_ID+cbATC_ID, &pszATC_TYPE, &cbATC_TYPE)) &&
					   SUCCEEDED(SimConnect_RetrieveString(pData, cbData, pszATC_TYPE+cbATC_TYPE, &pszTITLE, &cbTITLE)))
                    {
                       if (debug) printf("\nATC_ID = \"%s\" \nATC_TYPE = \"%s\" \nTITLE = \"%s\"\n",
                                pszATC_ID, pszATC_TYPE, pszTITLE );
       					strcpy_s(ATC_ID, pszATC_ID);
       					strcpy_s(ATC_TYPE, pszATC_TYPE);
       					strcpy_s(TITLE, pszTITLE);

                    } else
						if (debug) printf("\nCouldn't retrieve the aircraft strings.");
                    break;
                }

                case REQUEST_USER_POS:
				{
					// these events will come back once per second
					// from get_user_pos_updates() call
                    UserStruct *pU = (UserStruct*)&pObjData->dwData;
					user_pos.latitude = pU->latitude;
					user_pos.longitude = pU->longitude;
					user_pos.altitude = pU->altitude;
					user_pos.sim_on_ground = pU->sim_on_ground;
					user_pos.zulu_time = pU->zulu_time;
					user_pos.rpm = pU->rpm;
					// store position to igc log array on every nth tick
					if (++igc_tick_counter==IGC_TICK_COUNT) {
						if (debug) printf("B(%d,%d) ",int(user_pos.altitude), user_pos.rpm);
						igc_log_point(user_pos);
						igc_tick_counter = 0;
					}
					if (debug_events && user_pos.sim_on_ground!=0) printf(" [REQUEST_USER_POS (%d)G] ", igc_record_count);
					if (debug_events && user_pos.sim_on_ground==0) printf(" [REQUEST_USER_POS (%d)A] ", igc_record_count);
					// process 'on ground' status and decide whether to write a log file
					igc_ground_check(user_pos.sim_on_ground, user_pos.zulu_time);
                    break;
                }

                default:
					if (debug_info || debug) printf("\nUnknown SIMCONNECT_RECV_ID_SIMOBJECT_DATA request %d", pObjData->dwRequestID);
                    break;

            }
            break;
        }

        case SIMCONNECT_RECV_ID_EXCEPTION:
        {
            SIMCONNECT_RECV_EXCEPTION *except = (SIMCONNECT_RECV_EXCEPTION*)pData;
			if (debug_info || debug) printf("\n\n***** EXCEPTION=%d  SendID=%d  Index=%d  cbData=%d\n", except->dwException, except->dwSendID, except->dwIndex, cbData);
            break;
        }

        case SIMCONNECT_RECV_ID_OPEN:
        {
            SIMCONNECT_RECV_OPEN *open = (SIMCONNECT_RECV_OPEN*)pData;
			if (debug) printf("\nConnected to FSX Version %d.%d", open->dwApplicationVersionMajor, open->dwApplicationVersionMinor);
            break;
        }

		case SIMCONNECT_RECV_ID_EVENT_FILENAME:
        {
            SIMCONNECT_RECV_EVENT_FILENAME *evt = (SIMCONNECT_RECV_EVENT_FILENAME*)pData;
            switch(evt->uEventID)
            {
                case EVENT_FLIGHT:
					// FLT file loaded
					if (debug) printf("\n[ EVENT_FLIGHT ]: %s\n", evt->szFileName);
					// reset the IGC record count and start a new log
					igc_reset_log();
					// copy filename into flt_pathname global
					strcpy_s(flt_pathname, evt->szFileName);
					// build the wx_pathname global
					strcpy_s(wx_pathname, evt->szFileName);
					c_pointer = strrchr(wx_pathname, '.');
					if (c_pointer!=NULL) {
						c_pointer[1] = 'W';
						c_pointer[2] = 'X';
						c_pointer[3] = '\0';
					}
					// build the cmx_pathname global
					strcpy_s(cmx_pathname, evt->szFileName);
					c_pointer = strrchr(cmx_pathname, '.');
					if (c_pointer!=NULL) {
						c_pointer[1] = 'C';
						c_pointer[2] = 'M';
						c_pointer[3] = 'X';
						c_pointer[4] = '\0';
					}
					// build the xml_pathname global for mission xml file
					strcpy_s(xml_pathname, evt->szFileName);
					c_pointer = strrchr(xml_pathname, '.');
					if (c_pointer!=NULL) {
						c_pointer[1] = 'X';
						c_pointer[2] = 'M';
						c_pointer[3] = 'L';
						c_pointer[4] = '\0';
					}
					// calculate checksum for FLT, WX, CMX, XML files
					chksum_binary_file(chksum_flt, flt_pathname);
					if (chksum_binary_file(chksum_wx, wx_pathname)==CHKSUM_OK)
						wx_code = 1;
					chksum_binary_file(chksum_cmx, cmx_pathname);
					chksum_binary_file(chksum_xml, xml_pathname);
					get_startup_data();
                    break;

                case EVENT_AIRCRAFT:

					if (debug) printf("\n[ EVENT_AIRCRAFT ]: %s\n", evt->szFileName);
					// reset the IGC record count and start a new log
					igc_reset_log();
					// copy filename into flight_pathname global
					strcpy_s(air_pathname, evt->szFileName);
					// build the cfg_pathname global for aircraft.cfg
					strcpy_s(cfg_pathname, evt->szFileName);
					c_pointer = strrchr(cfg_pathname, '\\');
					if (c_pointer!=NULL) {
                        strcpy_s(c_pointer+1,30,"aircraft.cfg");
					}
					// calculate checksum for AIR and aircraft.cfg file
					chksum_binary_file(chksum_air, air_pathname);
					chksum_cfg_file(chksum_cfg, cfg_pathname);
					get_startup_data();
                    break;

                case EVENT_FLIGHTPLAN:

					if (debug) printf("\n[ EVENT_FLIGHTPLAN ]: %s\n", evt->szFileName);
					// reset the IGC record count and start a new log
					igc_reset_log();
					// copy filename into flight_pathname global
					strcpy_s(pln_pathname, evt->szFileName);
					pln_to_c(pln_pathname);
					//get_startup_data();
                    break;

                default:
		            if (debug_info || debug) printf("\nUnrecognized RECV_ID_EVENT_FILENAME Received:%d\n",evt->uEventID);
                   break;
            }
            break;
        }

        case SIMCONNECT_RECV_ID_QUIT:
        {
			// write the IGC file if there is one
			if (igc_record_count>IGC_MIN_RECORDS) {
				igc_write_file("autosave on quit");
				igc_reset_log();
			}
			// set flag to trigger a quit
            quit = 1;
            break;
        }

        default:
            if (debug_info || debug) printf("\nUnrecognized RECV_ID Received:%d\n",pData->dwID);
            break;
    }
}

void connectToSim()
{
    HRESULT hr;

	char sim_connect_string[100];

	sprintf_s(sim_connect_string, 
				sizeof(sim_connect_string), 
				"igc_logger v%.2f", 
				version);

    if (SUCCEEDED(SimConnect_Open(&hSimConnect, sim_connect_string, NULL, 0, 0, 0)))
    {
        if (debug_info || debug) printf("SimConnect_Open succeeded\n", version);   
          
        // Create some private events
        //hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_Z);
        //hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_X);

        // Link the private events to keyboard keys, and ensure the input events are off
        //hr = SimConnect_MapInputEventToClientEvent(hSimConnect, INPUT_ZX, "Z", EVENT_Z);
        //hr = SimConnect_MapInputEventToClientEvent(hSimConnect, INPUT_ZX, "X", EVENT_X);

        //hr = SimConnect_SetInputGroupState(hSimConnect, INPUT_ZX, SIMCONNECT_STATE_OFF);

        // Sign up for notifications
        //hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_ZX, EVENT_Z);
        //hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_ZX, EVENT_X);

		//*** CREATE ADD-ON MENU
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU);
		//hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_SHOW_TEXT);
		//hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_HIDE_TEXT);
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_MENU_WRITE_LOG);
		// Add sim_probe menu items
		hr = SimConnect_MenuAddItem(hSimConnect, "Sim_logger", EVENT_MENU, 0);
		//hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_MENU, "display status text", EVENT_MENU_SHOW_TEXT, 0);
		//hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_MENU, "hide status text", EVENT_MENU_HIDE_TEXT, 0);
		hr = SimConnect_MenuAddSubItem(hSimConnect, EVENT_MENU, "Save IGC log file", EVENT_MENU_WRITE_LOG, 0);
		// Sign up for the notifications
		hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_MENU, EVENT_MENU);
		hr = SimConnect_SetNotificationGroupPriority(hSimConnect, GROUP_MENU, SIMCONNECT_GROUP_PRIORITY_HIGHEST);

        // DEFINITION_AIRCRAFT
        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AIRCRAFT,
                                            "ATC ID", 
                                            NULL,
											SIMCONNECT_DATATYPE_STRINGV);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AIRCRAFT,
                                            "ATC TYPE", 
                                            NULL,
											SIMCONNECT_DATATYPE_STRINGV);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_AIRCRAFT,
                                            "TITLE", 
                                            NULL,
											SIMCONNECT_DATATYPE_STRINGV);

		// DEFINITION_STARTUP
        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU TIME", 
                                            "seconds",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU DAY OF MONTH", 
                                            "number",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU MONTH OF YEAR", 
                                            "number",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_STARTUP,
                                            "ZULU YEAR", 
                                            "number",
											SIMCONNECT_DATATYPE_INT32);

		// DEFINITION_USER_POS
        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "Plane Latitude", 
                                            "degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "Plane Longitude", 
                                            "degrees");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "PLANE ALTITUDE", 
                                            "meters");

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "SIM ON GROUND", 
                                            "bool",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "ZULU TIME", 
                                            "seconds",
											SIMCONNECT_DATATYPE_INT32);

        hr = SimConnect_AddToDataDefinition(hSimConnect, 
                                            DEFINITION_USER_POS,
                                            "GENERAL ENG RPM:1", 
                                            "Rpm",
											SIMCONNECT_DATATYPE_INT32);

		// Listen for the CumulusX.ReportSessionCode event
		hr = SimConnect_MapClientEventToSimEvent(hSimConnect, EVENT_CX_CODE, "CumulusX.ReportSessionCode");
		hr = SimConnect_AddClientEventToNotificationGroup(hSimConnect, GROUP_ZX, EVENT_CX_CODE, false);
		hr = SimConnect_SetNotificationGroupPriority(hSimConnect, GROUP_ZX, SIMCONNECT_GROUP_PRIORITY_DEFAULT);

        // Listen for a simulation start event
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_SIM_START, "SimStart");

        // Subscribe to the FlightLoaded event to detect flight start and end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_FLIGHT, "FlightLoaded");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_MISSIONCOMPLETED, "MissionCompleted");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_AIRCRAFT, "AircraftLoaded");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_FLIGHTPLAN, "FlightPlanActivated");

        // Subscribe to the MissionCompleted event to detect flight end
        hr = SimConnect_SubscribeToSystemEvent(hSimConnect, EVENT_WEATHER, "WeatherModeChanged");

		// Now loop checking for messages until quit
		hr  = S_OK;
        while( hr == S_OK && 0 == quit )
        {
            hr = SimConnect_CallDispatch(hSimConnect, MyDispatchProcSO, NULL);
            Sleep(1);
        } 
		if (hr==S_OK) hr = SimConnect_Close(hSimConnect);
		else {
			if (debug) printf("Fail code from CallDispatch\n");
			// write the IGC file if there is one
			if (igc_record_count>IGC_MIN_RECORDS) {
				igc_write_file("autosave on fsx crash");
			}
		}

	} else {
	    if (debug) printf("Couldn't connect to FSX.. logger will exit now\n");
	}
}


//int __cdecl _tmain(int argc, _TCHAR* argv[])
int main(int argc, char* argv[])
{
	bool no_flags = true;
	igc_reset_log();

	// set up command line arguments (debug mode)
	for (int i=1; i<argc; i++) {
		if (strcmp(argv[i],"debug")==0) {
			debug = true;
			debug_info = false;
		}
		else if (strcmp(argv[i],"info")==0)      debug_info = true; // will open console window
		else if (strcmp(argv[i],"calls")==0)     debug_calls = true;
		else if (strcmp(argv[i],"events")==0)    debug_events = true;
		else if (strncmp(argv[i],"log=",4)==0)   {
			igc_log_directory = argv[i]+4;
			no_flags = false;
		}
	}

    //debug
    //pln_to_c(argv[1]);
    //return 0;
    //debug end

	if (argc==2 && !debug && no_flags) {
		printf("\nChecking igc file checksum\n");
		Sleep(1000);printf(".");
		Sleep(1000);printf(".");
		Sleep(1000);printf(".\n");
		
		switch (check_file(argv[1]))
		{
			case CHKSUM_OK:
				printf("IGC file checks OK.\n");
				break;

			case CHKSUM_TOO_SHORT:
				printf("BAD CHECKSUM. This file contains a checksum but it is too short.\n");
				break;

			case CHKSUM_NOT_FOUND:
				printf("BAD CHECKSUM. This file does not contain a 'G' record.\n");
				break;

			case CHKSUM_BAD:
				printf("BAD CHECKSUM. 'G' record found but checksum is wrong.\n");
				break;

			case CHKSUM_FILE_ERROR:
				printf("FILE ERROR. Couldn't read the igc file \"%s\".\n", argv[1]);
				break;
		}

		return 0;
	}

	if (!debug && !debug_info) FreeConsole(); // kill console unless requested

	if (debug) {
		printf("Starting logger version %.2f in debug mode\n", version);
		printf("IGC logs folder '%s'\n", igc_log_directory);
		if (debug_info) printf("+info");
		if (debug_calls) printf("+calls");
		if (debug_events) printf("+events");
		//printf("\n");
		//chksum_string("jhsdfhsfkjhwefkjwfnm sdfmberfwnbefx");
		//chksum_to_string();
		//printf("\nchecksum = %s\n",chksum);
		//return 0;
	} else if (debug_info) {
		printf("Debug mode = debug_info\n");
	}

    connectToSim();
    return 0;
}