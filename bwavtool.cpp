#define _CRT_SECURE_NO_WARNINGS
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <climits>

#include <opus/opus.h>

#define PACKET_NIBBLES 16
#define PACKET_SAMPLES 14
#define PACKET_BYTES 8

#define NXOPUS_HEADER_ID  0x80000001u
#define NXOPUS_DATA_ID    0x80000004u

static long clamp16(long v) { return v <= -32768 ? -32768 : v >= 32767 ? 32767 : v; }
static long clamp_l(long v, long lo, long hi) { return v <= lo ? lo : v >= hi ? hi : v; }
static int getBytesForAdpcmSamples(int s) {
    int p = s / PACKET_SAMPLES, e = s % PACKET_SAMPLES;
    return PACKET_BYTES * p + (e ? (e / 2) + (e % 2) + 1 : 0);
}

static uint16_t readU16LE(const uint8_t* d) { return d[0] | (d[1] << 8); }
static uint32_t readU32LE(const uint8_t* d) { return d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24); }
static int16_t readI16LE(const uint8_t* d) { return (int16_t)readU16LE(d); }
static uint16_t readU16BE(const uint8_t* d) { return (d[0] << 8) | d[1]; }
static uint32_t readU32BE(const uint8_t* d) { return (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]; }
static int16_t readI16BE(const uint8_t* d) { return (int16_t)readU16BE(d); }

static void writeU16LE(uint8_t* d, uint16_t v) { d[0] = v & 0xFF; d[1] = v >> 8; }
static void writeU32LE(uint8_t* d, uint32_t v) { d[0] = v & 0xFF; d[1] = (v >> 8) & 0xFF; d[2] = (v >> 16) & 0xFF; d[3] = v >> 24; }
static void writeU16BE(uint8_t* d, uint16_t v) { d[0] = v >> 8; d[1] = v & 0xFF; }
static void writeU32BE(uint8_t* d, uint32_t v) { d[0] = v >> 24; d[1] = (v >> 16) & 0xFF; d[2] = (v >> 8) & 0xFF; d[3] = v & 0xFF; }

static const uint32_t crc32_tab[256] = {
0x00000000,0x77073096,0xee0e612c,0x990951ba,0x076dc419,0x706af48f,0xe963a535,0x9e6495a3,
0x0edb8832,0x79dcb8a4,0xe0d5e91e,0x97d2d988,0x09b64c2b,0x7eb17cbd,0xe7b82d07,0x90bf1d91,
0x1db71064,0x6ab020f2,0xf3b97148,0x84be41de,0x1adad47d,0x6ddde4eb,0xf4d4b551,0x83d385c7,
0x136c9856,0x646ba8c0,0xfd62f97a,0x8a65c9ec,0x14015c4f,0x63066cd9,0xfa0f3d63,0x8d080df5,
0x3b6e20c8,0x4c69105e,0xd56041e4,0xa2677172,0x3c03e4d1,0x4b04d447,0xd20d85fd,0xa50ab56b,
0x35b5a8fa,0x42b2986c,0xdbbbc9d6,0xacbcf940,0x32d86ce3,0x45df5c75,0xdcd60dcf,0xabd13d59,
0x26d930ac,0x51de003a,0xc8d75180,0xbfd06116,0x21b4f4b5,0x56b3c423,0xcfba9599,0xb8bda50f,
0x2802b89e,0x5f058808,0xc60cd9b2,0xb10be924,0x2f6f7c87,0x58684c11,0xc1611dab,0xb6662d3d,
0x76dc4190,0x01db7106,0x98d220bc,0xefd5102a,0x71b18589,0x06b6b51f,0x9fbfe4a5,0xe8b8d433,
0x7807c9a2,0x0f00f934,0x9609a88e,0xe10e9818,0x7f6a0dbb,0x086d3d2d,0x91646c97,0xe6635c01,
0x6b6b51f4,0x1c6c6162,0x856530d8,0xf262004e,0x6c0695ed,0x1b01a57b,0x8208f4c1,0xf50fc457,
0x65b0d9c6,0x12b7e950,0x8bbeb8ea,0xfcb9887c,0x62dd1ddf,0x15da2d49,0x8cd37cf3,0xfbd44c65,
0x4db26158,0x3ab551ce,0xa3bc0074,0xd4bb30e2,0x4adfa541,0x3dd895d7,0xa4d1c46d,0xd3d6f4fb,
0x4369e96a,0x346ed9fc,0xad678846,0xda60b8d0,0x44042d73,0x33031de5,0xaa0a4c5f,0xdd0d7cc9,
0x5005713c,0x270241aa,0xbe0b1010,0xc90c2086,0x5768b525,0x206f85b3,0xb966d409,0xce61e49f,
0x5edef90e,0x29d9c998,0xb0d09822,0xc7d7a8b4,0x59b33d17,0x2eb40d81,0xb7bd5c3b,0xc0ba6cad,
0xedb88320,0x9abfb3b6,0x03b6e20c,0x74b1d29a,0xead54739,0x9dd277af,0x04db2615,0x73dc1683,
0xe3630b12,0x94643b84,0x0d6d6a3e,0x7a6a5aa8,0xe40ecf0b,0x9309ff9d,0x0a00ae27,0x7d079eb1,
0xf00f9344,0x8708a3d2,0x1e01f268,0x6906c2fe,0xf762575d,0x806567cb,0x196c3671,0x6e6b06e7,
0xfed41b76,0x89d32be0,0x10da7a5a,0x67dd4acc,0xf9b9df6f,0x8ebeeff9,0x17b7be43,0x60b08ed5,
0xd6d6a3e8,0xa1d1937e,0x38d8c2c4,0x4fdff252,0xd1bb67f1,0xa6bc5767,0x3fb506dd,0x48b2364b,
0xd80d2bda,0xaf0a1b4c,0x36034af6,0x41047a60,0xdf60efc3,0xa867df55,0x316e8eef,0x4669be79,
0xcb61b38c,0xbc66831a,0x256fd2a0,0x5268e236,0xcc0c7795,0xbb0b4703,0x220216b9,0x5505262f,
0xc5ba3bbe,0xb2bd0b28,0x2bb45a92,0x5cb36a04,0xc2d7ffa7,0xb5d0cf31,0x2cd99e8b,0x5bdeae1d,
0x9b64c2b0,0xec63f226,0x756aa39c,0x026d930a,0x9c0906a9,0xeb0e363f,0x72076785,0x05005713,
0x95bf4a82,0xe2b87a14,0x7bb12bae,0x0cb61b38,0x92d28e9b,0xe5d5be0d,0x7cdcefb7,0x0bdbdf21,
0x86d3d2d4,0xf1d4e242,0x68ddb3f8,0x1fda836e,0x81be16cd,0xf6b9265b,0x6fb077e1,0x18b74777,
0x88085ae6,0xff0f6a70,0x66063bca,0x11010b5c,0x8f659eff,0xf862ae69,0x616bffd3,0x166ccf45,
0xa00ae278,0xd70dd2ee,0x4e048354,0x3903b3c2,0xa7672661,0xd06016f7,0x4969474d,0x3e6e77db,
0xaed16a4a,0xd9d65adc,0x40df0b66,0x37d83bf0,0xa9bcae53,0xdebb9ec5,0x47b2cf7f,0x30b5ffe9,
0xbdbdf21c,0xcabac28a,0x53b39330,0x24b4a3a6,0xbad03605,0xcdd70693,0x54de5729,0x23d967bf,
0xb3667a2e,0xc4614ab8,0x5d681b02,0x2a6f2b94,0xb40bbe37,0xc30c8ea1,0x5a05df1b,0x2d02ef8d
};

static uint32_t calcCRC32(const uint8_t* data, size_t len, uint32_t init) {
    uint32_t c = init; for (size_t i = 0; i < len; i++) c = crc32_tab[(c ^ data[i]) & 0xFF] ^ (c >> 8); return ~c;
}
static uint32_t calcCRC32(const uint8_t* data, size_t len) { return calcCRC32(data, len, 0xFFFFFFFF); }

typedef double tvec[3];
static void InnerProductMerge(tvec out, short p[14]) { for (int i = 0; i <= 2; i++) { out[i] = 0; for (int x = 0; x < 14; x++) out[i] -= p[x - i] * p[x]; } }
static void OuterProductMerge(tvec m[3], short p[14]) { for (int x = 1; x <= 2; x++) for (int y = 1; y <= 2; y++) { m[x][y] = 0; for (int z = 0; z < 14; z++) m[x][y] += p[z - x] * p[z - y]; } }
static bool AnalyzeRanges(tvec m[3], int* vi) {
    double r[3], v, t, mn, mx;
    for (int x = 1; x <= 2; x++) { v = std::max(fabs(m[x][1]), fabs(m[x][2])); if (v < 1e-15) return true; r[x] = 1.0 / v; }
    int mi = 0;
    for (int i = 1; i <= 2; i++) {
        for (int x = 1; x < i; x++) { t = m[x][i]; for (int y = 1; y < x; y++) t -= m[x][y] * m[y][i]; m[x][i] = t; }
        v = 0; for (int x = i; x <= 2; x++) { t = m[x][i]; for (int y = 1; y < i; y++) t -= m[x][y] * m[y][i]; m[x][i] = t; t = fabs(t) * r[x]; if (t >= v) { v = t; mi = x; } }
        if (mi != i) { for (int y = 1; y <= 2; y++) { t = m[mi][y]; m[mi][y] = m[i][y]; m[i][y] = t; } r[mi] = r[i]; }
        vi[i] = mi; if (m[i][i] == 0) return true;
        if (i != 2) { t = 1.0 / m[i][i]; for (int x = i + 1; x <= 2; x++) m[x][i] *= t; }
    }
    mn = 1e10; mx = 0; for (int i = 1; i <= 2; i++) { t = fabs(m[i][i]); if (t < mn) mn = t; if (t > mx) mx = t; } return mn / mx < 1e-10;
}
static void BidirectionalFilter(tvec m[3], int* vi, tvec o) {
    double t; int x = 0;
    for (int i = 1; i <= 2; i++) { int idx = vi[i]; t = o[idx]; o[idx] = o[i]; if (x) for (int y = x; y <= i - 1; y++) t -= o[y] * m[i][y]; else if (t != 0) x = i; o[i] = t; }
    for (int i = 2; i > 0; i--) { t = o[i]; for (int y = i + 1; y <= 2; y++) t -= o[y] * m[i][y]; o[i] = t / m[i][i]; } o[0] = 1.0;
}
static bool QuadraticMerge(tvec v) { double v2 = v[2], t = 1.0 - v2 * v2; if (t == 0) return true; v[0] = (v[0] - v2 * v2) / t; v[1] = (v[1] - v[1] * v2) / t; return fabs(v[1]) > 1.0; }
static void FinishRecord(tvec in, tvec out) { for (int z = 1; z <= 2; z++) { if (in[z] >= 1.0) in[z] = 0.9999999999; else if (in[z] <= -1.0) in[z] = -0.9999999999; } out[0] = 1.0; out[1] = (in[2] * in[1]) + in[1]; out[2] = in[2]; }
static void MatrixFilter(tvec src, tvec dst) {
    tvec m[3]; m[2][0] = 1.0; for (int i = 1; i <= 2; i++) m[2][i] = -src[i];
    for (int i = 2; i > 0; i--) { double v = 1.0 - (m[i][i] * m[i][i]); for (int y = 1; y <= i; y++) m[i - 1][y] = ((m[i][i] * m[i][y]) + m[i][y]) / v; }
    dst[0] = 1.0; for (int i = 1; i <= 2; i++) { dst[i] = 0; for (int y = 1; y <= i; y++) dst[i] += m[i][y] * dst[i - y]; }
}
static void MergeFinishRecord(tvec src, tvec dst) {
    tvec tmp; double v = src[0]; dst[0] = 1.0;
    for (int i = 1; i <= 2; i++) { double v2 = 0; for (int y = 1; y < i; y++) v2 += dst[y] * src[i - y]; dst[i] = v > 0 ? -(v2 + src[i]) / v : 0; tmp[i] = dst[i]; for (int y = 1; y < i; y++) dst[y] += dst[i] * dst[i - y]; v *= 1.0 - (dst[i] * dst[i]); }
    FinishRecord(tmp, dst);
}
static double ContrastVectors(tvec s1, tvec s2) {
    double v = (s2[2] * s2[1] + -s2[1]) / (1.0 - s2[2] * s2[2]);
    return (s1[0] * s1[0]) + (s1[1] * s1[1]) + (s1[2] * s1[2]) + (2.0 * v * ((s1[0] * s1[1]) + (s1[1] * s1[2]))) + (2.0 * (-s2[1] * v + -s2[2]) * s1[0] * s1[2]);
}
static void FilterRecords(tvec best[8], int exp, tvec* rec, int rc) {
    if (exp <= 0 || exp > 8) return;

    tvec bl[8];
    int b1[8];
    tvec b2;

    for (int x = 0; x < 2; x++) {
        memset(b1, 0, sizeof(b1));
        for (int y = 0; y < exp; y++) {
            for (int i = 0; i <= 2; i++) bl[y][i] = 0;
        }

        for (int z = 0; z < rc; z++) {
            int idx = 0;
            double val = 1e30;
            for (int i = 0; i < exp; i++) {
                double t = ContrastVectors(best[i], rec[z]);
                if (t < val) {
                    val = t;
                    idx = i;
                }
            }

            if (idx < 0 || idx >= exp) continue;

            b1[idx]++;
            MatrixFilter(rec[z], b2);
            for (int i = 0; i <= 2; i++) bl[idx][i] += b2[i];
        }

        for (int i = 0; i < exp; i++) {
            if (b1[i] > 0) {
                for (int y = 0; y <= 2; y++) bl[i][y] /= b1[i];
            }
        }

        for (int i = 0; i < exp; i++) MergeFinishRecord(bl[i], best[i]);
    }
}
static void DSPCorrelateCoefs(const short* src, int samples, short* coefs) {
    int nf = (samples + 13) / 14;
    short* bb = (short*)calloc(0x3800, sizeof(short));
    if (!bb) return;
    short ph[2][14] = {};
    tvec v1, v2, mt[3];
    tvec* rec = (tvec*)calloc(nf * 2, sizeof(tvec));
    if (!rec) { free(bb); return; }
    int rc = 0, vi[3];
    tvec best[8];
    for (int x = samples; x > 0;) {
        int fs = (x > 0x3800) ? 0x3800 : x;
        if (x <= 0x3800) {
            for (int z = 0; z < 14 && (fs + z) < 0x3800; z++) bb[fs + z] = 0;
        }
        x -= fs;
        if (fs > 0x3800) fs = 0x3800;
        memcpy(bb, src, fs * sizeof(short));
        src += fs;
        for (int i = 0; i < fs && i < 0x3800 - 14;) {
            for (int z = 0; z < 14; z++) ph[0][z] = ph[1][z];
            for (int z = 0; z < 14; z++) {
                if (i + z < fs) ph[1][z] = bb[i + z];
                else ph[1][z] = 0;
            }
            i += 14;
            InnerProductMerge(v1, ph[1]);
            if (fabs(v1[0]) > 10.0) {
                OuterProductMerge(mt, ph[1]);
                if (!AnalyzeRanges(mt, vi)) {
                    BidirectionalFilter(mt, vi, v1);
                    if (!QuadraticMerge(v1)) {
                        FinishRecord(v1, rec[rc]);
                        rc++;
                        if (rc >= nf * 2) rc = nf * 2 - 1;
                    }
                }
            }
        }
    }
    if (rc == 0) {
        for (int z = 0; z < 8; z++) {
            coefs[z * 2] = 0;
            coefs[z * 2 + 1] = 0;
        }
        free(rec);
        free(bb);
        return;
    }
    v1[0] = 1.0; v1[1] = 0; v1[2] = 0;
    for (int z = 0; z < rc; z++) {
        MatrixFilter(rec[z], best[0]);
        for (int y = 1; y <= 2; y++) v1[y] += best[0][y];
    }
    for (int y = 1; y <= 2; y++) v1[y] /= rc;
    MergeFinishRecord(v1, best[0]);
    int exp = 1;
    for (int w = 0; w < 3; w++) {
        v2[0] = 0; v2[1] = -1.0; v2[2] = 0;
        for (int i = 0; i < exp && (exp + i) < 8; i++) {
            for (int y = 0; y <= 2; y++) best[exp + i][y] = 0.01 * v2[y] + best[i][y];
        }
        exp = 1 << (w + 1);
        if (exp > 8) exp = 8;
        FilterRecords(best, exp, rec, rc);
    }
    for (int z = 0; z < 8; z++) {
        double d = -best[z][1] * 2048.0;
        coefs[z * 2] = (d > 32767) ? 32767 : (d < -32768) ? -32768 : (short)(long)lround(d);
        d = -best[z][2] * 2048.0;
        coefs[z * 2 + 1] = (d > 32767) ? 32767 : (d < -32768) ? -32768 : (short)(long)lround(d);
    }
    free(rec);
    free(bb);
}
static void DSPEncodeFrame(short p[16], int sc, uint8_t out[8], const short ci[8][2]) {
    int is[8][16], os[8][14], bi = 0, sc2[8];
    double da[8];
    for (int i = 0; i < 8; i++) {
        is[i][0] = p[0]; is[i][1] = p[1]; int dist = 0;
        for (int s = 0; s < sc; s++) {
            int v1 = ((p[s] * ci[i][1]) + (p[s + 1] * ci[i][0])) / 2048;
            int v2 = p[s + 2] - v1;
            int v3 = (v2 >= 32767) ? 32767 : (v2 <= -32768) ? -32768 : v2;
            if (abs(v3) > abs(dist)) dist = v3;
        }
        for (sc2[i] = 0; (sc2[i] <= 12) && ((dist > 7) || (dist < -8)); sc2[i]++, dist /= 2) {}
        sc2[i] = (sc2[i] <= 1) ? -1 : sc2[i] - 2;
        int idx = 0;
        do {
            sc2[i]++; da[i] = 0; idx = 0;
            for (int s = 0; s < sc; s++) {
                long long v1 = (long long)is[i][s] * ci[i][1] + (long long)is[i][s + 1] * ci[i][0];
                long long v2 = (((long long)p[s + 2] << 11) - v1) / 2048;
                int shift = sc2[i];
                long long divisor = 1LL << shift;
                int v3;
                if (v2 > 0) {
                    long long tmp = (v2 + divisor / 2) / divisor;
                    v3 = (tmp > 7) ? 7 : (tmp < -8) ? -8 : (int)tmp;
                }
                else {
                    long long tmp = (v2 - divisor / 2) / divisor;
                    v3 = (tmp > 7) ? 7 : (tmp < -8) ? -8 : (int)tmp;
                }
                if (v3 < -8) { int t = -8 - v3; if (idx < t) idx = t; v3 = -8; }
                else if (v3 > 7) { int t = v3 - 7; if (idx < t) idx = t; v3 = 7; }
                os[i][s] = v3;
                v1 = (v1 + ((v3 * (1LL << sc2[i])) << 11) + 1024) >> 11;
                is[i][s + 2] = (v1 >= 32767) ? 32767 : (v1 <= -32768) ? -32768 : (int)v1;
                long long diff = (long long)p[s + 2] - is[i][s + 2];
                da[i] += (double)diff * (double)diff;
            }
            for (int x = idx + 8; x > 256; x >>= 1) if (++sc2[i] >= 12) sc2[i] = 11;
        } while (sc2[i] < 12 && idx > 1);
    }
    double mn = 1e30;
    for (int i = 0; i < 8; i++) if (da[i] < mn) { mn = da[i]; bi = i; }
    for (int s = 0; s < sc; s++) p[s + 2] = (short)is[bi][s + 2];
    out[0] = (char)((bi << 4) | (sc2[bi] & 0xF));
    for (int s = sc; s < 14; s++) os[bi][s] = 0;
    for (int y = 0; y < 7; y++) out[y + 1] = (char)((os[bi][y * 2] << 4) | (os[bi][y * 2 + 1] & 0xF));
}

static void encodeADPCM(const std::vector<std::vector<int16_t>>& pcm, int16_t coefs[][16], std::vector<std::vector<uint8_t>>& adpcm, int totalSamples) {
    int nch = (int)pcm.size();
    for (int c = 0; c < nch; c++) {
        short c8[8][2]; for (int i = 0; i < 16; i++) c8[i / 2][i % 2] = coefs[c][i];
        int pk = totalSamples / PACKET_SAMPLES + (totalSamples % PACKET_SAMPLES != 0);
        adpcm[c].resize(getBytesForAdpcmSamples(totalSamples));
        size_t pos = 0;
        int16_t cs[16] = {};
        for (int p = 0; p < pk; p++) {
            memset(cs + 2, 0, PACKET_SAMPLES * sizeof(int16_t));
            int ns = std::min(totalSamples - p * PACKET_SAMPLES, PACKET_SAMPLES);
            for (int s = 0; s < ns; s++) cs[s + 2] = pcm[c][p * PACKET_SAMPLES + s];
            uint8_t blk[8];
            DSPEncodeFrame(cs, ns, blk, c8);
            cs[0] = cs[14]; cs[1] = cs[15];
            memcpy(&adpcm[c][pos], blk, getBytesForAdpcmSamples(ns));
            pos += getBytesForAdpcmSamples(ns);
        }
    }
}

static void decodeADPCM(const std::vector<std::vector<uint8_t>>& adpcm, int16_t coefs[][16], int totalSamples, std::vector<std::vector<int16_t>>& pcm) {
    int nch = (int)adpcm.size();
    pcm.resize(nch);
    for (int c = 0; c < nch; c++) {
        pcm[c].resize(totalSamples);
        const uint8_t* d = adpcm[c].data();
        int cps = d[0], yn1 = 0, yn2 = 0;
        size_t di = 0;
        for (int si = 0; si < totalSamples;) {
            if (si % 14 == 0) cps = d[di++];
            int out;
            if ((si++ & 1) == 0) out = d[di] >> 4; else out = d[di++] & 0x0F;
            if (out >= 8) out -= 16;
            int scale = 1 << (cps & 0x0F), ci2 = (cps >> 4) << 1;
            out = (0x400 + ((scale * out) << 11) + coefs[c][clamp_l(ci2, 0, 15)] * yn1 + coefs[c][clamp_l(ci2 + 1, 0, 15)] * yn2) >> 11;
            yn2 = yn1;
            yn1 = (int16_t)clamp16(out);
            pcm[c][si - 1] = yn1;
        }
    }
}

static void decodeOpusChannel(const uint8_t* nxopData, size_t nxopSize, int sr, std::vector<int16_t>& pcmOut) {
    uint32_t chunkId = readU32LE(nxopData);
    if (chunkId != NXOPUS_HEADER_ID) { fprintf(stderr, "NXOpus: bad header ID %#x\n", chunkId); return; }
    uint8_t channelCount = nxopData[9];
    uint32_t sampleRate = readU32LE(nxopData + 0x0C);
    uint32_t dataOffset = readU32LE(nxopData + 0x10);
    uint16_t preSkip = readU16LE(nxopData + 0x1C);

    const uint8_t* dcPtr = nxopData + dataOffset;
    uint32_t dcId = readU32LE(dcPtr);
    if (dcId != NXOPUS_DATA_ID) { fprintf(stderr, "NXOpus: bad data chunk ID %#x\n", dcId); return; }
    uint32_t dcSize = readU32LE(dcPtr + 4);
    const uint8_t* pktData = dcPtr + 8;

    int error;
    OpusDecoder* dec = opus_decoder_create(sampleRate, channelCount, &error);
    if (error != OPUS_OK) { fprintf(stderr, "opus_decoder_create: %s\n", opus_strerror(error)); return; }

    int frameSize = sampleRate / 50;
    std::vector<int16_t> frameBuf(frameSize * channelCount);

    size_t offset = 0;
    int skipLeft = preSkip;
    while (offset + 8 <= dcSize) {
        uint32_t pktSize = readU32BE(pktData + offset);
        offset += 8;
        if (pktSize == 0 || offset + pktSize > dcSize) break;

        int decoded = opus_decode(dec, pktData + offset, pktSize, frameBuf.data(), frameSize, 0);
        offset += pktSize;
        if (decoded < 0) { fprintf(stderr, "opus_decode: %s\n", opus_strerror(decoded)); break; }

        int samplesToWrite = decoded * channelCount;
        int skipSamples = skipLeft * channelCount;
        if (skipSamples > 0) {
            if (skipSamples >= samplesToWrite) { skipLeft -= decoded; continue; }
            samplesToWrite -= skipSamples;
            skipLeft = 0;
            pcmOut.insert(pcmOut.end(), frameBuf.data() + skipSamples, frameBuf.data() + skipSamples + samplesToWrite);
        }
        else {
            pcmOut.insert(pcmOut.end(), frameBuf.data(), frameBuf.data() + samplesToWrite);
        }
    }
    opus_decoder_destroy(dec);
}

static bool isValidOpusSampleRate(int sr) {
    return sr == 48000 || sr == 24000 || sr == 16000 || sr == 12000 || sr == 8000;
}

static std::vector<int16_t> resample(const std::vector<int16_t>& input, int fromRate, int toRate) {
    double ratio = (double)toRate / fromRate;
    int newLength = (int)(input.size() * ratio);
    std::vector<int16_t> output(newLength);
    for (int i = 0; i < newLength; i++) {
        double srcPos = i / ratio;
        int idx = (int)srcPos;
        double frac = srcPos - idx;
        if (idx + 1 < (int)input.size())
            output[i] = (int16_t)lround(input[idx] * (1.0 - frac) + input[idx + 1] * frac);
        else if (idx < (int)input.size())
            output[i] = input[idx];
    }
    return output;
}

struct OpusEncodedPacket {
    std::vector<uint8_t> data;
    uint32_t finalRange = 0;
};

static std::vector<uint8_t> encodeOpusChannel(const int16_t* pcm, int sampleCount, int sr, int& preSkipOut) {
    int actualSr = sr;
    std::vector<int16_t> resampled;

    if (!isValidOpusSampleRate(sr)) {
        resampled = resample(std::vector<int16_t>(pcm, pcm + sampleCount), sr, 48000);
        pcm = resampled.data();
        sampleCount = (int)resampled.size();
        actualSr = 48000;
    }

    int error;
    OpusEncoder* enc = opus_encoder_create(actualSr, 1, OPUS_APPLICATION_AUDIO, &error);
    if (error != OPUS_OK) { fprintf(stderr, "opus_encoder_create: %s\n", opus_strerror(error)); return {}; }
    opus_encoder_ctl(enc, OPUS_SET_BITRATE(96000));
    opus_encoder_ctl(enc, OPUS_SET_VBR(1));
    opus_encoder_ctl(enc, OPUS_SET_VBR_CONSTRAINT(0));
    opus_encoder_ctl(enc, OPUS_GET_LOOKAHEAD(&preSkipOut));

    int frameDurationMs = 20;
    int frameSize = (actualSr / 1000) * frameDurationMs;
    uint8_t buf[1275];

    std::vector<OpusEncodedPacket> packets;
    for (int i = 0; i + frameSize <= sampleCount; i += frameSize) {
        int nb = opus_encode(enc, pcm + i, frameSize, buf, sizeof(buf));
        if (nb < 0) { fprintf(stderr, "opus_encode: %s\n", opus_strerror(nb)); break; }
        OpusEncodedPacket pkt;
        pkt.data.assign(buf, buf + nb);
        opus_encoder_ctl(enc, OPUS_GET_FINAL_RANGE(&pkt.finalRange));
        packets.push_back(std::move(pkt));
    }
    opus_encoder_destroy(enc);

    uint32_t dcSize = 0;
    for (auto& p : packets) dcSize += 8 + (uint32_t)p.data.size();

    std::vector<uint8_t> result(32 + 8 + dcSize, 0);
    size_t pos = 0;
    auto wb = [&](const void* s, size_t n) { memcpy(&result[pos], s, n); pos += n; };
    auto wu32le = [&](uint32_t v) { uint8_t b[4]; writeU32LE(b, v); wb(b, 4); };
    auto wu16le = [&](uint16_t v) { uint8_t b[2]; writeU16LE(b, v); wb(b, 2); };
    auto wu32be = [&](uint32_t v) { uint8_t b[4]; writeU32BE(b, v); wb(b, 4); };

    wu32le(NXOPUS_HEADER_ID);
    wu32le(24);
    uint8_t ver = 0; wb(&ver, 1);
    uint8_t ch = 1; wb(&ch, 1);
    wu16le(0);
    wu32le(actualSr);
    wu32le(32);
    wu32le(0);
    wu32le(0);
    wu16le((uint16_t)preSkipOut);
    wu16le(0);

    wu32le(NXOPUS_DATA_ID);
    wu32le(dcSize);
    for (auto& p : packets) {
        wu32be((uint32_t)p.data.size());
        wu32be(p.finalRange);
        wb(p.data.data(), p.data.size());
    }
    return result;
}

struct WavInfo {
    int nch = 0, sr = 0, bps = 0;
    size_t samples = 0;
    std::vector<std::vector<int16_t>> pcm;
};
static bool readWav(const char* path, WavInfo& w) {
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return false; }
    uint8_t h[44];
    if (fread(h, 1, 44, f) != 44) { fclose(f); return false; }
    if (memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVEfmt ", 8)) { fprintf(stderr, "Not a valid WAV\n"); fclose(f); return false; }
    if (readU32LE(h + 16) != 16 || readU16LE(h + 20) != 1) { fprintf(stderr, "Only PCM WAV supported\n"); fclose(f); return false; }
    w.nch = readU16LE(h + 22);
    w.sr = readU32LE(h + 24);
    w.bps = readU16LE(h + 34);
    if (w.bps != 16) { fprintf(stderr, "Only 16-bit WAV supported\n"); fclose(f); return false; }
    size_t off = 36;
    fseek(f, 0, SEEK_END);
    size_t fsz = ftell(f);
    while (off + 4 < fsz) {
        fseek(f, (long)off, SEEK_SET);
        uint8_t tmp[4];
        if (fread(tmp, 1, 4, f) == 4 && !memcmp(tmp, "data", 4)) break;
        off++;
    }
    fseek(f, (long)off, SEEK_SET);
    uint8_t db[8];
    if (fread(db, 1, 8, f) != 8) { fclose(f); return false; }
    uint32_t dataSz = readU32LE(db + 4);
    w.samples = dataSz / w.nch / 2;
    w.pcm.resize(w.nch);
    for (int c = 0; c < w.nch; c++) w.pcm[c].resize(w.samples);
    std::vector<uint8_t> raw(dataSz);
    if (fread(raw.data(), 1, dataSz, f) != dataSz) { fclose(f); return false; }
    for (size_t i = 0; i < w.samples; i++) for (int c = 0; c < w.nch; c++) w.pcm[c][i] = readI16LE(&raw[i * (2 * w.nch) + c * 2]);
    fclose(f);
    return true;
}
static bool writeWav(const char* path, const WavInfo& w) {
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); return false; }
    uint32_t dataSz = (uint32_t)(w.samples * w.nch * 2);
    uint32_t fileSz = 36 + dataSz;
    uint8_t h[44] = { 0 };
    memcpy(h, "RIFF", 4);
    writeU32LE(h + 4, fileSz);
    memcpy(h + 8, "WAVEfmt ", 8);
    writeU32LE(h + 16, 16);
    writeU16LE(h + 20, 1);
    writeU16LE(h + 22, (uint16_t)w.nch);
    writeU32LE(h + 24, w.sr);
    writeU32LE(h + 28, w.sr * w.nch * 2);
    writeU16LE(h + 32, (uint16_t)(w.nch * 2));
    writeU16LE(h + 34, (uint16_t)w.bps);
    memcpy(h + 36, "data", 4);
    writeU32LE(h + 40, dataSz);
    size_t written = fwrite(h, 1, 44, f);
    if (written != 44) { fclose(f); return false; }
    std::vector<uint8_t> raw(dataSz);
    for (size_t i = 0; i < w.samples; i++) {
        for (int c = 0; c < w.nch; c++) {
            writeU16LE(&raw[i * (2 * w.nch) + c * 2], (uint16_t)w.pcm[c][i]);
        }
    }
    written = fwrite(raw.data(), 1, dataSz, f);
    if (written != dataSz) { fclose(f); return false; }
    fclose(f);
    return true;
}

struct BwavInfo {
    int nch = 0, sr = 0, codec = 0;
    bool loopFlag = false;
    uint32_t loopStart = 0;
    size_t samples = 0;
    std::vector<std::vector<int16_t>> pcm;
    std::vector<std::vector<uint8_t>> adpcm;
    std::vector<std::vector<uint8_t>> opusNx;
    int16_t coefs[16][16] = {};
};

static bool readBwav(const char* path, BwavInfo& b) {
    FILE* f = fopen(path, "rb");
    if (!f) { perror(path); return false; }
    fseek(f, 0, SEEK_END);
    size_t fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> data(fsz);
    if (fread(data.data(), 1, fsz, f) != fsz) { fclose(f); return false; }
    fclose(f);
    if (memcmp(data.data(), "BWAV", 4)) { fprintf(stderr, "Not a valid BWAV\n"); return false; }
    bool BOM = (readI16BE(&data[4]) == -257);
    b.nch = BOM ? readU16BE(&data[0x0E]) : readU16LE(&data[0x0E]);
    b.codec = (BOM ? readU16BE(&data[0x10]) : readU16LE(&data[0x10])) + 1;
    b.sr = BOM ? readU32BE(&data[0x14]) : readU32LE(&data[0x14]);
    b.samples = BOM ? readU32BE(&data[0x18]) : readU32LE(&data[0x18]);
    b.loopFlag = ((int32_t)(BOM ? readU32BE(&data[0x4C]) : readU32LE(&data[0x4C])) != -1);
    b.loopStart = BOM ? readU32BE(&data[0x50]) : readU32LE(&data[0x50]);
    if (b.codec == 2) for (int c = 0; c < b.nch; c++) for (int i = 0; i < 16; i++) b.coefs[c][i] = BOM ? readI16BE(&data[0x20 + i * 2 + c * 0x4C]) : readI16LE(&data[0x20 + i * 2 + c * 0x4C]);
    b.pcm.resize(b.nch);
    b.adpcm.resize(b.nch);
    b.opusNx.resize(b.nch);
    for (int c = 0; c < b.nch; c++) {
        uint32_t chOff = BOM ? readU32BE(&data[0x40 + c * 0x4C]) : readU32LE(&data[0x40 + c * 0x4C]);
        if (b.codec == 1) {
            b.pcm[c].resize(b.samples);
            for (size_t s = 0; s < b.samples; s++) b.pcm[c][s] = BOM ? readI16BE(&data[chOff + s * 2]) : readI16LE(&data[chOff + s * 2]);
        }
        else if (b.codec == 2) {
            int totalBytes = getBytesForAdpcmSamples((int)b.samples);
            b.adpcm[c].resize(totalBytes);
            memcpy(b.adpcm[c].data(), &data[chOff], totalBytes);
        }
        else if (b.codec == 3) {
            if (chOff + 8 > fsz) { fprintf(stderr, "Channel %d offset out of range\n", c); return false; }
            uint32_t dcOff = readU32LE(&data[chOff + 0x10]);
            uint32_t dcSize = readU32LE(&data[chOff + dcOff + 4]);
            size_t nxSize = dcOff + 8 + dcSize;
            b.opusNx[c].resize(nxSize);
            memcpy(b.opusNx[c].data(), &data[chOff], nxSize);
        }
    }
    if (b.codec == 2) decodeADPCM(b.adpcm, b.coefs, (int)b.samples, b.pcm);
    if (b.codec == 3) {
        for (int c = 0; c < b.nch; c++) {
            decodeOpusChannel(b.opusNx[c].data(), b.opusNx[c].size(), b.sr, b.pcm[c]);
        }
    }
    return true;
}

static bool writeBwav(const char* path, const BwavInfo& b) {
    int nch = b.nch;
    int actualSr = b.sr;
    size_t actualSamples = b.samples;

    std::vector<std::vector<uint8_t>> adpcm(nch);
    std::vector<std::vector<uint8_t>> opusNx(nch);
    std::vector<std::vector<int16_t>> resampledPcm;
    uint32_t dataBytes = 0;

    if (b.codec == 1) {
        dataBytes = (uint32_t)(b.samples * 2);
    }
    else if (b.codec == 2) {
        int16_t coefs[16][16];
        memcpy(coefs, b.coefs, sizeof(coefs));
        encodeADPCM(b.pcm, coefs, adpcm, (int)b.samples);
        dataBytes = (uint32_t)adpcm[0].size();
    }
    else if (b.codec == 3) {
        if (!isValidOpusSampleRate(b.sr)) {
            resampledPcm.resize(nch);
            for (int c = 0; c < nch; c++)
                resampledPcm[c] = resample(b.pcm[c], (int)b.sr, 48000);
            actualSr = 48000;
            actualSamples = resampledPcm[0].size();
        }

        const std::vector<std::vector<int16_t>>& opusPcm = resampledPcm.empty() ? b.pcm : resampledPcm;

        for (int c = 0; c < nch; c++) {
            int preSkip = 0;
            opusNx[c] = encodeOpusChannel(opusPcm[c].data(), (int)opusPcm[c].size(), actualSr, preSkip);
            if (opusNx[c].empty()) { fprintf(stderr, "Opus encode failed for channel %d\n", c); return false; }
        }
        dataBytes = 0;
        for (int c = 0; c < nch; c++) if ((uint32_t)opusNx[c].size() > dataBytes) dataBytes = (uint32_t)opusNx[c].size();
    }

    uint32_t alignData = (dataBytes + 0x3F) & ~0x3Fu;
    uint32_t hdrSize = (0x10 + 0x4C * nch + 0x3F) & ~0x3Fu;
    std::vector<uint32_t> chOff(nch);
    for (int c = 0; c < nch; c++)
        chOff[c] = hdrSize + (uint32_t)c * alignData;
    size_t fsz = chOff[nch - 1] + alignData;
    std::vector<uint8_t> buf(fsz, 0);
    size_t bufpos = 0;
    auto wb = [&](const void* src, size_t len) { memcpy(&buf[bufpos], src, len); bufpos += len; };
    auto wu16 = [&](uint16_t v) { uint8_t b2[2]; writeU16LE(b2, v); wb(b2, 2); };
    auto wu32 = [&](uint32_t v) { uint8_t b4[4]; writeU32LE(b4, v); wb(b4, 4); };
    wb("BWAV", 4);
    { uint8_t bom[2] = { 0xFF, 0xFE }; wb(bom, 2); }
    { uint8_t ver[2] = { 0x01, 0x00 }; wb(ver, 2); }
    wu32(0);
    { uint8_t pad[2] = { 0x00, 0x00 }; wb(pad, 2); }
    wu16(nch);
    for (int c = 0; c < nch; c++) {
        wu16(b.codec - 1);
        wu16(nch > 1 ? (c % 2) : 2);
        wu32((uint32_t)actualSr);
        wu32((uint32_t)actualSamples);
        wu32((uint32_t)actualSamples);
        if (b.codec == 2) { for (int i = 0; i < 16; i++) wu16((uint16_t)b.coefs[c][i]); }
        else { for (int i = 0; i < 8; i++) wu32(0); }
        wu32(chOff[c]);
        wu32(chOff[c]);
        wu32(1);
        if (b.loopFlag) wu32((uint32_t)actualSamples); else wu32(0xFFFFFFFF);
        wu32(b.loopStart);
        wu16(b.codec == 2 ? adpcm[c][0] : 0);
        wu16(0);
        wu16(0);
        wu16(0);
    }
    std::vector<uint8_t> crcbuf;
    for (int c = 0; c < nch; c++) {
        bufpos = chOff[c];
        if (b.codec == 2) {
            memcpy(&buf[bufpos], adpcm[c].data(), dataBytes);
            crcbuf.insert(crcbuf.end(), adpcm[c].data(), adpcm[c].data() + dataBytes);
        }
        else if (b.codec == 3) {
            memcpy(&buf[bufpos], opusNx[c].data(), opusNx[c].size());
            crcbuf.insert(crcbuf.end(), opusNx[c].data(), opusNx[c].data() + opusNx[c].size());
        }
        else {
            for (size_t s = 0; s < b.samples; s++) {
                uint8_t tmp[2];
                writeU16LE(tmp, (uint16_t)b.pcm[c][s]);
                buf[bufpos++] = tmp[0];
                buf[bufpos++] = tmp[1];
                crcbuf.push_back(tmp[0]);
                crcbuf.push_back(tmp[1]);
            }
        }
    }
    uint32_t crc = calcCRC32(crcbuf.data(), crcbuf.size(), 0xFFFFFFFF);
    { uint8_t tmp[4]; writeU32LE(tmp, crc); memcpy(&buf[8], tmp, 4); }
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); return false; }
    fwrite(buf.data(), 1, buf.size(), f);
    fclose(f);
    return true;
}

static std::string autoOutput(const char* in, const char* ext) {
    std::string s(in);
    size_t p = s.rfind('.');
    if (p != std::string::npos) s = s.substr(0, p);
    s += ext;
    return s;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
            "Usage: %s -e input.wav [-c pcm16|dsp|opus] [output.bwav]\n"
            "       %s -d input.bwav [output.wav]\n"
            "  -c  codec: pcm16 (raw 16-bit), dsp (ADPCM, default), opus\n",
            argv[0], argv[0]);
        return 1;
    }

    const char* mode = argv[1];
    int codec = 2;
    const char* in = NULL;
    const char* out = NULL;

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            const char* cname = argv[++i];
            if (strcmp(cname, "pcm16") == 0)      codec = 1;
            else if (strcmp(cname, "dsp") == 0)   codec = 2;
            else if (strcmp(cname, "opus") == 0)  codec = 3;
            else { fprintf(stderr, "Unknown codec: %s (use pcm16, dsp, or opus)\n", cname); return 1; }
        }
        else if (!in) {
            in = argv[i];
        }
        else if (!out) {
            out = argv[i];
        }
    }
    if (!in) { fprintf(stderr, "Missing input file\n"); return 1; }

    if (strcmp(mode, "-e") == 0) {
        WavInfo w;
        if (!readWav(in, w)) return 1;
        std::string outpath = out ? std::string(out) : autoOutput(in, ".bwav");

        BwavInfo b;
        b.nch = (int)w.nch;
        b.sr = w.sr;
        b.codec = codec;
        b.loopFlag = false;
        b.loopStart = 0;
        b.samples = w.samples;
        b.pcm = w.pcm;
        memset(b.coefs, 0, sizeof(b.coefs));
        if (codec == 2) {
            for (int c = 0; c < b.nch; c++)
                DSPCorrelateCoefs(w.pcm[c].data(), (int)w.samples, b.coefs[c]);
        }
        if (!writeBwav(outpath.c_str(), b)) {
            fprintf(stderr, "Error: failed to write %s\n", outpath.c_str());
            return 1;
        }
        const char* cname = codec == 1 ? "PCM16" : codec == 2 ? "DSPADPCM" : "OPUS";
        printf("Encoded %s -> %s (%zu samples, %d ch, %d Hz, %s)\n",
            in, outpath.c_str(), w.samples, w.nch, w.sr, cname);
    }
    else if (strcmp(mode, "-d") == 0) {
        BwavInfo b;
        if (!readBwav(in, b)) return 1;
        std::string outpath = out ? std::string(out) : autoOutput(in, ".wav");

        WavInfo w;
        w.nch = b.nch;
        w.sr = b.sr;
        w.bps = 16;
        w.samples = b.samples;
        w.pcm = b.pcm;
        if (!writeWav(outpath.c_str(), w)) return 1;
        printf("Decoded %s -> %s (%zu samples, %d ch, %d Hz)\n",
            in, outpath.c_str(), b.samples, b.nch, b.sr);
    }
    else {
        fprintf(stderr, "Unknown mode: %s\n", mode);
        return 1;
    }
    return 0;
}