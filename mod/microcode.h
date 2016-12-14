#ifndef DISABLE_MICROCODE_LOAD /* For Copyright reasons */
#define DISABLE_MICROCODE_LOAD
/* I don't know what it is, but it might be firmware loaded into the
   Xilinx PLD, or just some kind of Finite State Machine, as Grant
   said in one of his postings.
*/

static u_int8_t Microcode[]={ /* 1853 bytes */
      0xff, 0x04, 0xc0, 0x79, 0xf6, 0xe4, 0xef, 0xff, 0xf9, 0xff, 0xff,
      0xff, 0xfb, 0xff, 0xb7, 0xf9, 0x7b, 0xfb, 0xfb, 0xfb, 0xfb, 0xfb,
      0xbb, 0xbf, 0xb9, 0xbf, 0xbf, 0xbf, 0xb7, 0xb7, 0xbf, 0xbf, 0xfe,
      0xf1, 0xff, 0xff, 0xef, 0xeb, 0xff, 0xdf, 0xef, 0xff, 0xef, 0xfb,
      0xfb, 0xfb, 0xff, 0x7b, 0xfb, 0xda, 0xfb, 0x7e, 0xff, 0xff, 0xf7,
      0xf6, 0xfb, 0xf7, 0xff, 0xfb, 0xff, 0xfb, 0xda, 0xfb, 0xfb, 0xfb,
      0xff, 0xdb, 0xef, 0xfe, 0x9f, 0xdf, 0x9f, 0xdf, 0xdf, 0xdf, 0xcf,
      0x9f, 0xdf, 0xfe, 0x7e, 0xf9, 0x99, 0xfe, 0xb9, 0x3c, 0xf9, 0x7c,
      0xff, 0xe7, 0xff, 0x7d, 0xed, 0xef, 0x5d, 0x7d, 0xdd, 0x7f, 0xbf,
      0xcf, 0xeb, 0xeb, 0xcf, 0xef, 0xef, 0xef, 0xe7, 0xff, 0xf9, 0xfe,
      0xfe, 0xcc, 0xee, 0xbe, 0xde, 0xec, 0xfe, 0xee, 0xff, 0xf7, 0xf5,
      0xef, 0xd5, 0xf7, 0xf5, 0xf7, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xfd, 0xff, 0x7b, 0x5f, 0xf2, 0xb3, 0xad, 0xff, 0xd1,
      0xe8, 0xaf, 0xdc, 0xff, 0xe9, 0xff, 0xcd, 0xfd, 0x6d, 0x87, 0xfa,
      0xfd, 0xfe, 0x7b, 0xfd, 0x3d, 0xfd, 0x3f, 0xfd, 0xee, 0x9f, 0xf7,
      0xdf, 0x6d, 0xdf, 0xd9, 0xff, 0xf9, 0xf7, 0xff, 0xbd, 0xff, 0x8f,
      0x2d, 0x8f, 0xff, 0xbb, 0x3e, 0xff, 0xff, 0xfd, 0x77, 0x78, 0x7b,
      0xf8, 0xff, 0xf9, 0xff, 0xff, 0xcf, 0xff, 0xf3, 0x4f, 0x73, 0x7f,
      0xf7, 0xe1, 0xff, 0x75, 0xfd, 0x9f, 0xff, 0x9f, 0xff, 0xbf, 0x0f,
      0xff, 0xdf, 0xeb, 0xff, 0xdb, 0xea, 0xb2, 0x9f, 0xea, 0xfd, 0xff,
      0x5e, 0xff, 0xf7, 0xf7, 0xf5, 0xff, 0xf7, 0xb7, 0xff, 0xfd, 0xbe,
      0xad, 0xbd, 0x9f, 0xbd, 0xbd, 0xbd, 0x9d, 0x7d, 0xf7, 0xff, 0x7f,
      0xf7, 0xfe, 0xfe, 0xff, 0xfe, 0xff, 0xbd, 0xf2, 0xed, 0x97, 0xf6,
      0xf5, 0xdf, 0xed, 0xf6, 0xeb, 0xb1, 0x7f, 0xee, 0xfe, 0xff, 0x2f,
      0x7a, 0xb5, 0xff, 0xef, 0xff, 0xcb, 0xff, 0xff, 0x7f, 0xfb, 0xf7,
      0xfd, 0x7e, 0xfe, 0x5f, 0xff, 0xff, 0xff, 0xdf, 0xdf, 0xff, 0xbf,
      0x3b, 0x7f, 0xb6, 0xe7, 0xff, 0xbf, 0xf6, 0xf7, 0x7f, 0xdf, 0xff,
      0xf6, 0x9f, 0xdf, 0xff, 0xd2, 0xdf, 0xff, 0xff, 0xfc, 0xaf, 0x3f,
      0xdf, 0xff, 0xbf, 0xbe, 0xfe, 0xff, 0xf7, 0x7f, 0xec, 0xe1, 0xff,
      0xdf, 0xed, 0x6d, 0xff, 0xbf, 0xff, 0xef, 0xfe, 0x77, 0xfb, 0xeb,
      0xef, 0xff, 0xfb, 0xfd, 0x7f, 0xdf, 0xff, 0xff, 0x7f, 0x4f, 0xff,
      0xef, 0xaf, 0xf3, 0xfd, 0xff, 0xf1, 0xb3, 0xfe, 0xb3, 0xfe, 0x7f,
      0x59, 0xff, 0xfb, 0x5f, 0xdd, 0x7f, 0x8b, 0xff, 0xe7, 0xfb, 0xf6,
      0xb6, 0xff, 0xe7, 0xf6, 0xf5, 0xfb, 0xff, 0xdf, 0x65, 0xf7, 0xff,
      0x6f, 0xf7, 0xb7, 0x9b, 0xff, 0xeb, 0xfa, 0x9f, 0x32, 0x46, 0x87,
      0xfe, 0xff, 0xd1, 0xff, 0xf7, 0xef, 0xe5, 0xd1, 0xef, 0xe9, 0xef,
      0xff, 0xff, 0x3f, 0xcf, 0x8f, 0xcf, 0xef, 0xcf, 0xbd, 0xcd, 0xff,
      0xdd, 0x7d, 0x7e, 0x5e, 0x7e, 0x5e, 0xbc, 0x7f, 0xfe, 0xff, 0xaf,
      0xea, 0xef, 0xe9, 0xeb, 0xeb, 0xef, 0xc9, 0xff, 0x7a, 0xdd, 0xcb,
      0x5f, 0xdf, 0xdd, 0xdf, 0x7e, 0xf7, 0xfe, 0xfb, 0xf7, 0xf6, 0x72,
      0xf7, 0xf6, 0xb6, 0xb2, 0xf6, 0xdd, 0xfd, 0xff, 0xdf, 0xff, 0xfb,
      0xfb, 0xdf, 0xff, 0xfe, 0x9a, 0x7f, 0xbf, 0xa6, 0xef, 0x5f, 0xbf,
      0xfa, 0xbf, 0xc7, 0xb5, 0xe9, 0xfa, 0xf5, 0xff, 0xb9, 0xd7, 0xff,
      0xbb, 0xcf, 0x1f, 0xe7, 0x8f, 0xff, 0xef, 0xef, 0xff, 0xf7, 0x79,
      0x5e, 0x7d, 0x7f, 0xfe, 0x7f, 0x3f, 0xbf, 0xff, 0xee, 0xdf, 0xdb,
      0xfb, 0xc6, 0xff, 0xdb, 0xfb, 0xff, 0x7f, 0xf7, 0xa7, 0xdf, 0xbd,
      0x6f, 0x5f, 0xdf, 0xff, 0xff, 0xf3, 0xfe, 0xf5, 0xfe, 0xff, 0x7b,
      0xfa, 0x7e, 0xff, 0x5c, 0xbf, 0x97, 0xf7, 0x9d, 0xff, 0x31, 0xf3,
      0xfb, 0xf5, 0xfa, 0xbe, 0xbf, 0xbe, 0xfe, 0xbf, 0xbf, 0xdf, 0x6f,
      0xe7, 0xfd, 0xf3, 0xfd, 0xbd, 0xff, 0xdd, 0xf5, 0xff, 0x3b, 0xdf,
      0xf5, 0x07, 0xff, 0x07, 0x3d, 0xdf, 0x07, 0xdb, 0x6d, 0x7d, 0x4f,
      0x7f, 0x7d, 0xfe, 0xff, 0x7e, 0xfc, 0xef, 0xdf, 0xd9, 0x7f, 0x9b,
      0xbf, 0xff, 0x5f, 0x91, 0x7f, 0xf7, 0xfd, 0xdf, 0xfd, 0xfe, 0xef,
      0xff, 0x9d, 0xff, 0xfb, 0xfe, 0xfd, 0xfa, 0xfa, 0xfa, 0x5c, 0xff,
      0xdb, 0xdf, 0xaf, 0xa7, 0xff, 0x83, 0xf7, 0xe5, 0xb7, 0xbf, 0xff,
      0x3c, 0x1e, 0xae, 0x3f, 0x96, 0x3f, 0x7f, 0xde, 0x7f, 0xf7, 0xf8,
      0xfb, 0x71, 0xfd, 0xf4, 0xf9, 0xfa, 0xf7, 0xbd, 0xae, 0xab, 0xaf,
      0xaf, 0x2f, 0xbb, 0xaf, 0xbf, 0xee, 0x75, 0x6f, 0x7f, 0x7f, 0x7f,
      0x7b, 0x3d, 0xff, 0xfe, 0xef, 0xde, 0xdd, 0xdf, 0xdb, 0xdf, 0xcb,
      0xdb, 0xdb, 0x77, 0xef, 0xff, 0xff, 0xf7, 0xef, 0x6f, 0xef, 0xe6,
      0xdf, 0xab, 0xfe, 0xfe, 0xbe, 0xff, 0x7e, 0xfa, 0xde, 0xfe, 0x5e,
      0xd7, 0xd7, 0xd5, 0xff, 0xd5, 0xe3, 0xdf, 0x97, 0xef, 0x3e, 0x7c,
      0x3e, 0xfe, 0xbf, 0xbc, 0x3f, 0xff, 0xff, 0xe7, 0x7d, 0xf9, 0x71,
      0xff, 0xfd, 0xfd, 0xf9, 0xff, 0xbb, 0xff, 0xa7, 0x1f, 0xff, 0x6f,
      0xef, 0x6f, 0x6f, 0xf7, 0x7d, 0xfd, 0x4f, 0xbf, 0x7f, 0x7d, 0xff,
      0x7c, 0xfd, 0x6f, 0xbb, 0x7f, 0xff, 0xff, 0xe7, 0xfb, 0xa3, 0xe1,
      0x7f, 0xfd, 0x4d, 0x7f, 0xfe, 0x4f, 0xd6, 0xd7, 0x96, 0xfe, 0xf3,
      0xf2, 0x78, 0xfa, 0xff, 0xfa, 0xbe, 0xea, 0xff, 0xdf, 0xc6, 0xcb,
      0xcf, 0xff, 0xcf, 0x77, 0xe7, 0xbf, 0xff, 0x34, 0xdb, 0x97, 0xfb,
      0x7f, 0xef, 0xef, 0xff, 0xff, 0xf7, 0xf1, 0xff, 0x3f, 0xff, 0x7d,
      0x5f, 0xf7, 0xff, 0xbf, 0x7f, 0xfd, 0x5b, 0xf7, 0xed, 0xff, 0xaf,
      0x7c, 0xff, 0xf9, 0xfb, 0x6b, 0xff, 0xeb, 0xfe, 0xdb, 0xfd, 0xef,
      0xef, 0xef, 0x7f, 0xe3, 0xdf, 0xb7, 0xff, 0xdf, 0xce, 0x7e, 0xff,
      0xfe, 0x1f, 0xfa, 0xd5, 0xff, 0xdf, 0xff, 0xfe, 0xd3, 0x5c, 0xff,
      0xfe, 0xff, 0xde, 0xff, 0xdb, 0xf5, 0xdd, 0xc7, 0xff, 0xe7, 0xfd,
      0xef, 0xfd, 0xfd, 0xff, 0xf7, 0xba, 0xee, 0xbf, 0xfe, 0xb7, 0xfe,
      0xbb, 0x9e, 0xbc, 0xd7, 0xfd, 0xff, 0xfd, 0xff, 0xf3, 0xfe, 0xf5,
      0xfd, 0x3f, 0x7f, 0x6f, 0x7f, 0x6b, 0x2f, 0x6f, 0x3f, 0xef, 0xdf,
      0xbd, 0xbf, 0xfb, 0xff, 0xcf, 0xbd, 0xbf, 0xdf, 0xfd, 0xaf, 0xfb,
      0xfb, 0xfb, 0xb7, 0x2a, 0xfb, 0xba, 0xaf, 0x7a, 0x5d, 0x77, 0x5f,
      0x5f, 0x59, 0x7d, 0x5e, 0xff, 0xfd, 0xfb, 0xb1, 0xf3, 0xf8, 0xff,
      0xfc, 0xfe, 0xff, 0xff, 0x9f, 0xe7, 0xf7, 0xd7, 0xff, 0xc5, 0xf7,
      0xff, 0xfd, 0xef, 0x3e, 0xff, 0xad, 0x2e, 0xfd, 0xbf, 0x9f, 0xf9,
      0xef, 0xd7, 0xf3, 0xf7, 0x7a, 0xf9, 0xf6, 0xfd, 0xff, 0xf6, 0xbf,
      0xbd, 0x7f, 0x77, 0xcf, 0xbf, 0xef, 0xdf, 0xff, 0xff, 0x55, 0xff,
      0xf9, 0x7f, 0xfa, 0x7d, 0x7f, 0xfb, 0xff, 0xcf, 0xeb, 0xcf, 0xef,
      0xff, 0xf3, 0xe9, 0xff, 0x9e, 0x7d, 0xfe, 0xff, 0x5f, 0xfd, 0x9e,
      0xdf, 0xff, 0xfe, 0xff, 0x7b, 0x6b, 0x7b, 0xfc, 0x5f, 0xbb, 0xfa,
      0xba, 0xeb, 0xdc, 0xb7, 0x36, 0xe6, 0xfc, 0x3f, 0x97, 0x92, 0x9f,
      0xef, 0xfa, 0xff, 0xff, 0xeb, 0xfd, 0xad, 0xbd, 0xfd, 0xdf, 0xf7,
      0xf7, 0xf7, 0xff, 0x7f, 0xff, 0xf5, 0xf5, 0xff, 0xbf, 0x4f, 0x45,
      0x4c, 0xaf, 0xbf, 0x69, 0x67, 0x8f, 0xfb, 0x7d, 0x7e, 0x7e, 0xf6,
      0xf8, 0x71, 0x7b, 0x73, 0xfe, 0xcf, 0xf7, 0xf7, 0x7e, 0xe7, 0xef,
      0xfb, 0xfb, 0xf3, 0x77, 0x9f, 0x9f, 0xff, 0xdf, 0xff, 0xd7, 0xdf,
      0x1f, 0xdf, 0x6b, 0xfc, 0xfe, 0xfe, 0xfa, 0xfa, 0xfa, 0x72, 0xfa,
      0xde, 0xf6, 0xeb, 0xef, 0xf7, 0xf7, 0xf7, 0xb6, 0xb7, 0xfb, 0xfc,
      0x9d, 0x9d, 0xbd, 0xfd, 0xbd, 0xbf, 0xfd, 0x5d, 0xf7, 0x7f, 0xff,
      0xef, 0xbf, 0xef, 0xff, 0xfe, 0xff, 0xbd, 0xee, 0xde, 0xdc, 0xee,
      0xef, 0xdf, 0xd2, 0xee, 0xef, 0x61, 0xfe, 0x7d, 0xfd, 0x7d, 0xfd,
      0xfd, 0x3d, 0xfd, 0xee, 0xf3, 0xcb, 0xff, 0xf7, 0xef, 0xff, 0xff,
      0xf3, 0x7f, 0x8e, 0x3f, 0xff, 0x1f, 0xef, 0xf7, 0xff, 0xd7, 0xff,
      0xfb, 0xec, 0xae, 0xac, 0xf4, 0x73, 0x6c, 0x6c, 0xf7, 0xdf, 0xc7,
      0xef, 0xe7, 0xeb, 0xff, 0xe7, 0xe7, 0xcf, 0xfe, 0xfc, 0xfc, 0x1f,
      0x5d, 0xfd, 0x3f, 0x3d, 0x7f, 0xe6, 0xd7, 0xff, 0x7f, 0xd9, 0x6d,
      0xed, 0xd8, 0xf9, 0xe7, 0xbd, 0xff, 0xcb, 0xff, 0xef, 0xbb, 0xff,
      0xf7, 0xaf, 0xef, 0xf5, 0x7f, 0xfe, 0xff, 0xfe, 0xbd, 0xff, 0x7f,
      0xfe, 0x8e, 0xf7, 0xed, 0xf3, 0xc2, 0x03, 0x8d, 0xca, 0xf7, 0x7f,
      0xdf, 0x7f, 0x4e, 0x9d, 0x9b, 0x7f, 0x3f, 0xaf, 0xff, 0xfb, 0xff,
      0xff, 0xe6, 0x77, 0xfc, 0xf2, 0x74, 0x76, 0x5f, 0x9d, 0xdf, 0x67,
      0xff, 0x77, 0x74, 0xe3, 0xf7, 0xff, 0x3e, 0xd5, 0x2c, 0xaf, 0x3e,
      0xfa, 0x14, 0x7e, 0xff, 0xb7, 0xf9, 0x7f, 0xec, 0xab, 0xf1, 0xe7,
      0xe1, 0xeb, 0x3f, 0xdf, 0x3f, 0xeb, 0xcd, 0x8f, 0xbf, 0xdf, 0xab,
      0xdf, 0x7d, 0xbf, 0x7f, 0x7e, 0x5c, 0xfe, 0x7f, 0x7f, 0x7f, 0xaf,
      0xf9, 0xff, 0xeb, 0xcf, 0xab, 0xeb, 0xf7, 0xad, 0x7a, 0xbd, 0x7f,
      0xbf, 0xdf, 0xcf, 0xdf, 0xdf, 0xdf, 0xff, 0xfb, 0xb6, 0xf2, 0xf6,
      0xf6, 0xb7, 0xf6, 0xf6, 0xf6, 0xdf, 0xfd, 0xff, 0xbf, 0xfd, 0xff,
      0xdf, 0xfd, 0xff, 0xf7, 0x5a, 0xfb, 0xbb, 0xbf, 0xbf, 0xff, 0x9a,
      0xbf, 0xb7, 0xd7, 0xf5, 0x75, 0xf7, 0xf9, 0xf4, 0xdf, 0xe5, 0xf9,
      0xbd, 0x2f, 0x89, 0x8f, 0x0f, 0xcf, 0xff, 0x0f, 0xcf, 0xff, 0xf9,
      0xfe, 0x5c, 0x7e, 0x5e, 0xdc, 0x7f, 0x7e, 0xfe, 0xee, 0xbb, 0xab,
      0xcb, 0xd9, 0xd3, 0xff, 0xd3, 0xd6, 0x7f, 0xbf, 0xef, 0x3f, 0xff,
      0x1d, 0xef, 0x17, 0xaf, 0xf7, 0xbb, 0xfd, 0xff, 0xf9, 0x7f, 0xf0,
      0xff, 0xfc, 0xfd, 0x5f, 0xf7, 0xff, 0x97, 0xb7, 0x87, 0xdf, 0xb7,
      0xaf, 0xff, 0x7c, 0x2f, 0x1f, 0x3f, 0x1f, 0xef, 0x3f, 0x3d, 0xff,
      0xf7, 0xfd, 0xe9, 0xf9, 0xf8, 0xf1, 0xff, 0x79, 0xd1, 0x3f, 0xbe,
      0xbf, 0xe7, 0xfb, 0xfc, 0xd7, 0xff, 0xcf, 0xef, 0xf5, 0xeb, 0x79,
      0xff, 0x9f, 0xef, 0xc6, 0x7f, 0xfe, 0xee, 0x5f, 0xff, 0xdb, 0xff,
      0xd3, 0xd7, 0x7f, 0xd3, 0x7d, 0xff, 0xff, 0xdb, 0xff, 0x9f, 0xbf,
      0xff, 0x9e, 0xff, 0xfb, 0xf8, 0xff, 0xee, 0xb7, 0xf4, 0xfb, 0xaf,
      0xda, 0xdf, 0x87, 0xff, 0x36, 0xfd, 0xa7, 0x9d, 0xfe, 0xb7, 0xf5,
      0x3c, 0xfe, 0xbf, 0xff, 0xff, 0xf7, 0xf6, 0xb7, 0x7c, 0xf7, 0xf9,
      0xff, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xfd, 0xbd, 0xbe, 0xfb, 0xbf,
      0xff, 0xaf, 0xa7, 0xff, 0xb7, 0xef, 0x7d, 0xff, 0x7f, 0xff, 0x5f,
      0x7f, 0xdf, 0x7f, 0xff, 0xef, 0xdb, 0xdb, 0xdb, 0xd9, 0xfb, 0xdb,
      0xd8, 0xdb, 0x77, 0xf7, 0xef, 0xef, 0xef, 0x73, 0xf3, 0xff, 0xfb,
      0xf7, 0xeb, 0xfe, 0x3e, 0xff, 0xee, 0xfb, 0x9f, 0xff, 0xfe, 0x1e,
      0xe7, 0xd3, 0xff, 0x57, 0xff, 0xff, 0xff, 0xd7, 0xff, 0xbe, 0xbf,
      0xff, 0x7f, 0xfc, 0xff, 0xff, 0x7f, 0xf7, 0xe7, 0xfd, 0xfd, 0x7f,
      0xf5, 0xff, 0xff, 0xff, 0xfc, 0xbb, 0x6f, 0x6f, 0xff, 0xff, 0xf7,
      0xff, 0xff, 0x6f, 0xfb, 0x79, 0x3f, 0xd7, 0xff, 0xfd, 0xb5, 0xaf,
      0xdf, 0xfe, 0x4f, 0xfb, 0xfb, 0xff, 0xef, 0xff, 0x7e, 0xff, 0xf3,
      0x7f, 0xdd, 0xde, 0xfe, 0xef, 0xf7, 0xef, 0xff, 0x9f, 0xde, 0xab,
      0xbe, 0xfe, 0xff, 0xfe, 0xf7, 0xbf, 0xff, 0xf8, 0xde, 0xf3, 0xf6,
      0xbf, 0xed, 0xff, 0xff, 0xbf, 0xc7, 0xef, 0xfe, 0xff, 0x3f, 0x7f,
      0xff, 0xff, 0xaf, 0x7f, 0xbf, 0xf7, 0xff, 0xff, 0xf8, 0xbd, 0x9f,
      0xff, 0x1b, 0xfb, 0xbb, 0xff, 0xff, 0xff, 0x6d, 0xff, 0xff, 0x5f,
      0xd7, 0xff, 0xfd, 0xff, 0xf7, 0xbf, 0xfc, 0xff, 0x5f, 0xff, 0xbe,
      0xef, 0xff, 0x7f, 0xff, 0x95, 0xcf, 0x7d, 0xfb, 0xf7, 0x7e, 0xff,
      0xff, 0xf7, 0xdf, 0x7e, 0xff, 0xbf, 0xbe, 0xff, 0xf3, 0xff, 0xff,
      0xfc, 0xfe, 0x7b, 0xff, 0xfd, 0xfd, 0xdd, 0xff, 0xff, 0x67, 0xef,
      0xfb, 0xff, 0xf7, 0xeb, 0xf7, 0xfa, 0xff, 0xbf, 0xba, 0xfa, 0xfd,
      0xbf, 0xfa, 0xbe, 0xf7, 0xff, 0xff, 0xbb, 0xfd, 0xf7, 0xde, 0xfc,
      0xfc, 0xbf, 0x6f, 0x6f, 0x6f, 0x6e, 0xaf, 0x6f, 0x6f, 0x6f, 0xdf,
      0xf5, 0xff, 0xff, 0xfd, 0xf9, 0xff, 0xff, 0xff, 0x2f, 0xaf, 0xff,
      0xff, 0xbb, 0xed, 0xfb, 0xfb, 0xff, 0xff, 0x7b, 0xfe, 0xff, 0x5f,
      0xff, 0x4f, 0x5f, 0xfe, 0x5f, 0xbb, 0xfb, 0xff, 0xff, 0xf1, 0xf3,
      0xfc, 0x7c, 0x7f, 0x7f, 0x9f, 0xff, 0xff, 0xd7, 0xdf, 0xe7, 0xe7,
      0xff, 0xff, 0xed, 0xfe, 0xff, 0x9f, 0xf9, 0x3f, 0xff, 0xfd, 0x3f,
      0xfd, 0xd7, 0xff, 0xfe, 0xdb, 0xff, 0xf9, 0xe7, 0xff, 0xf9, 0xbf,
      0xff, 0xff, 0xdf, 0xff, 0xff, 0xff, 0xf7, 0xcf, 0xfb, 0xf5, 0xff,
      0x7f, 0xfb, 0xff, 0xff, 0xf9, 0x7f, 0x7a, 0xef, 0xff, 0xff, 0xfb,
      0xfd, 0xff, 0xf3, 0xff, 0xef, 0x7f, 0xfd, 0xff, 0xbe, 0xff, 0xef,
      0x9a, 0xff, 0x7f, 0x9f, 0xfb, 0x7e, 0xff, 0x7a, 0xff, 0xff, 0x7f,
      0x7f, 0xfe, 0x5f, 0xb7, 0xbb, 0xb7, 0xaf, 0xbf, 0xbf, 0xbf, 0xb7,
      0xff, 0xba, 0xbc, 0xfd, 0x9d, 0xbd, 0xb9, 0xbd, 0x9d, 0xdd, 0xf7,
      0xfb, 0xfa, 0xfa, 0xfa, 0xfa, 0xfa, 0xfa, 0xf8, 0xbe, 0xbf, 0x9d,
      0xbf, 0xdf, 0x9f, 0xbf, 0x97, 0xff, 0xfb, 0x75, 0xfe, 0x7e, 0xfe,
      0xfe, 0xfe, 0xfe, 0x7e, 0xfe, 0xef, 0xfd, 0xff, 0xfb, 0xfd, 0xff,
      0xff, 0xfd, 0xfd, 0xff, 0xff
      };

#define LONGDELAY	5

void
frpw_load_microcode(struct ppcd_softc *sc)
{
int t, i;
u_int8_t s, p;

s=rc(); s&=0x12; s|=0xC;
wc(2);
wc(4); DELAY(LONGDELAY);
wc(0); wc(4); wc(1); wc(6);

for (i=0; i < sizeof(Microcode); i++) {
  p=Microcode[i];
  for (t=0; t<=7; t++) {
    wd(p&1);
    p>>=1;
    wc(4); DELAY(LONGDELAY); wc(6);
    }
  }

for (i=0; i<=2; i++) {
  wd(1);
  wc(4); DELAY(LONGDELAY); wc(6);
  }

wc(s);
}

#else /* PARANOID Copyright owners */
void frpw_load_microcode(struct ppcd_softc *sc) { /* stub */ }
#endif /* DISABLE_MICROCODE_LOAD */