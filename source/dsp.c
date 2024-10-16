#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "fsl_powerquad.h"
#include "fsl_i2c.h"

#include "dsp.h"
#include "funcs.h"
#include "sysdep_pcm.h"
#include "OA_algorithm.h"
#include "mma8652fc.h"
#include "stack.h"
#include "io.h"

/****************************************************************
 *	filter(b,a,x,yini)
 ****************************************************************/
static I2C_Type* i2c;
static i2c_master_handle_t i2c_handle;
static uint32_t mma8652_cfg;

static volatile status_t i2c_res;

#define MMA8652_ADDR				0x1D

#define MMA8652_STATUS      		0x00
#define MMA8652_OUT_X_MSB			0x01
#define MMA8652_OUT_X_LSB			0x02
#define MMA8652_OUT_Y_MSB			0x03
#define MMA8652_OUT_Y_LSB			0x04
#define MMA8652_OUT_Z_MSB			0x05
#define MMA8652_OUT_Z_LSB			0x06
#define MMA8652_F_SETUP				0x09
#define MMA8652_TRIG_CFG			0x0A
#define MMA8652_SYSMOD				0x0B
#define MMA8652_INT_SOURCE			0x0C
#define MMA8652_WHOAMI				0x0D
#define MMA8652_XYZ_DATA_CFG		0x0E
#define MMA8652_CTRL_REG1			0x2A
#define MMA8652_CTRL_REG2			0x2B
#define MMA8652_CTRL_REG3			0x2C
#define MMA8652_CTRL_REG4			0x2D
#define MMA8652_CTRL_REG5			0x2E
#define MMA8652_RES_8				(2)
#define MMA8652_ID 					0x4A
#define MMA8652_SCALE_8G			(2<<16)
#define MMA8652_RATE_1_56			(7<<3)
#define MMA8652_DATA_READY			(8)

static void i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
   i2c_res = status;
}

/* mma8652_init
 *   initializes the sensor according to 'cfg'
 */

header* mfilter (Calc *cc, header *hd)
// filter as in Matlab (almost)
{
	header *hdb=hd, *hda, *hdx, *hdy, *result;
	real *ma,*mb,*mx,*my;
	int ra,ca,rb,cb,rx,cx,ry,cy;
	
	hda=next_param(cc,hdb); hdx=next_param(cc,hda);
	hdy=next_param(cc,hdx);
	hdb=getvalue(cc,hdb); hda=getvalue(cc,hda);
	hdx=getvalue(cc,hdx); hdy=getvalue(cc,hdy);
	getmatrix(hdb,&rb,&cb,&mb); getmatrix(hda,&ra,&ca,&ma);
	getmatrix(hdx,&rx,&cx,&mx); getmatrix(hdy,&ry,&cy,&my);

	if (ra!=1 || rb!=1 || rx!=1 || ry!=1) cc_error(cc,"Filter can only work with row vectors");
	if (cx<=cb) cc_error(cc,"data must be longer then filter");
	if (cy<ca-1) cc_error(cc,"not enough start values for feedback filter.\n");
	if (ca<1 || cb<1) cc_error(cc,"filter coefficients must not be empty");
	
	if (isreal(hda) && isreal(hdb) && isreal(hdx) && isreal(hdy)) {
		if (ma[0]==0) cc_error(cc,"first denominator coeff must be non-zero");

		result=new_matrix(cc,1,cx-cb+1,"");
		real *mres=matrixof(result);
		for (int i=0; i<cx-cb+1; i++) {
			real s=0.0;
			for (int j=0; j<cb; j++) s+=mb[j]*mx[cb-1+i-j];
			for (int j=1; j<ca; j++) {
				if (i-j>=0) {
					s-=mres[i-j]*ma[j];
				} else {
					s-=my[i-j+cy]*ma[j];
				}
			}
			s=s/ma[0];
			mres[i]=s;
		}
	} else if (isrealorcplx(hda) && isrealorcplx(hdb) && 
		isrealorcplx(hdx) && isrealorcplx(hdy)) {
		real r=ma[0]*ma[0]+ma[1]*ma[1];
		if (r==0) cc_error(cc,"first denominator coeff must be non-zero");
		result=new_cmatrix(cc,1,cx-cb+1,"");
/*		real *mres=matrixof(result);
		for (int i=0; i<cx-cb+1; i++) {
			real sre=0,sim=0;
			for (int j=0; j<cb; j++) {
				sre+=getvre(hdb,j)*getvre(hdx,cb-1+i-j)-getvim(hdb,j)*getvim(hdx,cb-1+i-j);
				sim+=getvim(hdb,j)*getvre(hdx,cb-1+i-j)+getvre(hdb,j)*getvim(hdx,cb-1+i-j);
			}
			for (int j=1; j<ca; j++) {
				if (i-j>=0) {
					sre-=getvre(result,i-j)*getvre(hda,j)-getvim(result,i-j)*getvim(hda,j);
					sim-=getvim(result,i-j)*getvre(hda,j)+getvre(result,i-j)*getvim(hda,j);
				} else {
					sre-=getvre(hdy,i-j+cy)*getvre(hda,j)-getvim(hdy,i-j+cy)*getvim(hda,j);
					sim-=getvim(hdy,i-j+cy)*getvre(hda,j)+getvre(hdy,i-j+cy)*getvim(hda,j);
				}
			}
			sre=(sre*getvre(hda,0)+sim*getvim(hda,0))/r;
			sim=(sim*getvre(hda,0)-sre*getvim(hda,0))/r;
			mres[2*i]=sre; mres[2*i+1]=sim;
		}
*/
	} else cc_error(cc,"bad parameters in filter(b,a,x,yini)");

	return moveresult(cc,hd,result);
}

/****************************************************************
 *	FFT
 ****************************************************************/
int nn;
cplx *ff,*zz,*fh;

static void rfft (LONG m0, LONG p0, LONG q0, LONG n)
/***** rfft 
	make a fft on x[m],x[m+q0],...,x[m+(p0-1)*q0] (p points).
	one has xi_p0 = xi_n^n = zz[n] ; i.e., p0*n=nn.
*****/
{	LONG p,q,m,l;
	LONG mh,ml;
	int found=0;
	cplx sum,h;
	if (p0==1) return;
//	if (test_key()==escape) cc_error("fft interrupted")
	if (p0%2==0) {
		p=p0/2; q=2;
	} else {
		q=3;
		while (q*q<=p0) {
			if (p0%q==0) {
				found=1; break;
			}
			q+=2;
		}
		if (found) p=p0/q;
		else { q=p0; p=1; }
	}
	if (p>1) for (m=0; m<q; m++) 
		rfft((m0+m*q0)%nn,p,q*q0,nn/p);
	mh=m0;
	for (l=0; l<p0; l++) {
		ml=l%p;
		c_copy(ff[(m0+ml*q*q0)%nn],sum);
		for (m=1; m<q; m++) {
			c_mul(ff[(m0+(m+ml*q)*q0)%nn],zz[(n*l*m)%nn],h);
			c_add(sum,h,sum);
		}
		sum[0]/=q; sum[1]/=q;
		c_copy(sum,fh[mh]);
		mh+=q0; if (mh>=nn) mh-=nn;
	}
	for (l=0; l<p0; l++) {
		c_copy(fh[m0],ff[m0]);
		m0+=q0; if (m0>=nn) mh-=nn;
	}
}

static void fft (Calc *cc, real *a, int n, int signum)
{	cplx z;
	real h;
	int i;
	
	if (n==0) return;
	nn=n;

	char *ram=cc->newram;
	ff=(cplx *)a;
	zz=(cplx *)ram;
	ram+=n*sizeof(cplx);
	fh=(cplx *)ram;
	ram+=n*sizeof(cplx);
	if (ram>cc->udfstart)  cc_error(cc,"Memory overflow!");
	
	/* compute zz[k]=e^{-2*pi*i*k/n}, k=0,...,n-1 */
	h=2*M_PI/n; z[0]=cos(h); z[1]=signum*sin(h);
	zz[0][0]=1; zz[0][1]=0;
	for (i=1; i<n; i++) {
		if (i%16) { zz[i][0]=cos(i*h); zz[i][1]=signum*sin(i*h); }
		else c_mul(zz[i-1],z,zz[i]);
	}
	rfft(0,n,1,1);
	if (signum==-1)
		for (i=0; i<n; i++) {
			ff[i][0]*=n; ff[i][1]*=n;
		}
}

header* mfft (Calc *cc, header *hd)
{	header *st=hd,*result;
	real *m,*mr;
	int r,c;
	hd=getvalue(cc,hd);
	if (hd->type==s_real || hd->type==s_matrix)	{
		make_complex(cc,st); hd=st;
	}
	getmatrix(hd,&r,&c,&m);
	if (r!=1) cc_error(cc,"row vector expected");
	result=new_cmatrix(cc,1,c,"");
	mr=matrixof(result);
    memmove((char *)mr,(char *)m,(ULONG)2*c*sizeof(real));
	fft(cc,mr,c,-1);
	return pushresults(cc,result);
}

header* mifft (Calc *cc, header *hd)
{	header *st=hd,*result;
	real *m,*mr;
	int r,c;
	hd=getvalue(cc,hd);
	if (hd->type==s_real || hd->type==s_matrix) {
		make_complex(cc,st); hd=st;
	}
	getmatrix(hd,&r,&c,&m);
	if (r!=1) cc_error(cc,"row vector expected");
	result=new_cmatrix(cc,1,c,"");
	mr=matrixof(result);
    memmove((char *)mr,(char *)m,(ULONG)2*c*sizeof(real));
	fft(cc,mr,c,1);
	return pushresults(cc,result);
}

/* accelerometer */


#define I2C4_MASTER_CLK 12000000

header* maccel(Calc *cc, header *hd) {
    // Initialisation de l'I2C et du capteur MMA8652FC


    // Lire les données de l'accéléromètre
    uint8_t status;
    mma8652_status(&status);
    if (status & MMA8652_DATA_READY) {
        int32_t data[3];
        mma8652_read_xyz(data);
        // Retourner les données en tant que tableau ou matrice
        header *result = new_matrix(cc, 1, 3, "");
        real *mres = matrixof(result);
        mres[0] = (real)data[0]/1000;
        mres[1] = (real)data[1]/1000;
        mres[2] = (real)data[2]/1000;
        return pushresults(cc, result);
    } else {
        cc_error(cc, "No data ready from the accelerometer.");
    }

    return NULL;
}


#define MAX_VECTOR_SIZE 128 // Taille maximale pour les vecteurs
#define TYPE_FLOAT 1
#define TYPE_VECTOR 2

#include <time.h> // Pour la mesure du temps

header* mpqcos(Calc* cc, header* hd) {

	if (hd->type != s_real && hd->type != s_matrix) {
		cc_error(cc, "Invalid type for mpqcos. Expected real or vector.");
	return NULL;

	}

	 if (hd->type == s_real) {
		 real* input = realof(hd);
		 float result;
		 PQ_CosF32((float*)input, &result);
		 header* res_hd = new_real(cc, (real)result, "");
		 return pushresults(cc, res_hd);
	 } else if (hd->type == s_matrix) {
		 dims* dim = dimsof(hd);
		 int rows = dim->r;
		 int cols = dim->c;
		 int length = rows * cols;
		 real* input = matrixof(hd);
		 header* res_hd = new_matrix(cc, rows, cols, "");
		 real* output = matrixof(res_hd);
		 PQ_VectorCosF32((float*)input, (float*)output, length);
		 return pushresults(cc, res_hd);
	 	 }

	 return NULL;

	}


static int32_t fft_out[1024]; // 512 points
static int32_t fft_in[1024];

header* mpqfft(Calc *cc, header *hd)
{
    header *result;
    real *input_data, *output_data;
    int r, c;
    hd = getvalue(cc, hd);
	pq_config_t PQ_config_fft;
    PQ_GetDefaultConfig(&PQ_config_fft);
    PQ_config_fft.inputAFormat = kPQ_32Bit;
    PQ_config_fft.inputAPrescale = 9; // 1<<9 = 512 = fftsize
    PQ_config_fft.inputBFormat = kPQ_32Bit;
    PQ_config_fft.inputBPrescale = 0;
    PQ_config_fft.tmpFormat = kPQ_32Bit;
    PQ_config_fft.tmpPrescale = 0;
    PQ_config_fft.outputFormat = kPQ_32Bit;
    PQ_config_fft.outputPrescale = 0;
    PQ_config_fft.machineFormat= kPQ_32Bit;
    PQ_SetConfig(POWERQUAD, &PQ_config_fft);


    if (hd->type == s_real)
    {
        getmatrix(hd, &r, &c, &input_data);
        for (int i = 0; i < r*c; ++i) {
            fft_in[i] = (int32_t)(input_data[i]);
        }

        result = new_real(cc, 0, "");
        output_data = matrixof(result);

        PQ_TransformRFFT(POWERQUAD, r*c, fft_in, fft_out);
        PQ_WaitDone(POWERQUAD);

        for (int i = 0; i < r*c; ++i) {
            output_data[i] = (real)(fft_out[i])/16;
        }

        return pushresults(cc, result);
    }

    if (hd->type == s_matrix)
    {
        getmatrix(hd, &r, &c, &input_data);

        if (r != 1)
        {
            cc_error(cc, "row vector expected");
            return NULL;
        }

        for (int i = 0; i < c; ++i) {
            fft_in[i*2] = (int32_t)(input_data[i]);
        }

        result = new_cmatrix(cc, 1, c, "");
        output_data = matrixof(result);

        PQ_TransformCFFT(POWERQUAD, c, fft_in, fft_out);
        PQ_WaitDone(POWERQUAD);

        for (int i = 0; i < r*c*2; ++i) {
            output_data[i] = (real)(fft_out[i])/16;
        }

        return pushresults(cc, result);
    }

    cc_error(cc, "unsupported data type");
    return NULL;
}


header* mpqifft(Calc* cc, header* hd)
{
    header *result;
    real *input_data, *output_data;
    int r, c;

    // Récupérer les données
    hd = getvalue(cc, hd);

    // Cas réel : appliquer IFFT réelle
    if (hd->type == s_real)
    {
        real input_value;
        real output_value;

        getmatrix(hd, &r, &c, &input_data);
        input_value = *input_data;

        // Allouer mémoire pour le résultat
        result = new_real(cc, 0, "");
        output_data = matrixof(result);

        // Appliquer la IFFT réelle
        PQ_TransformIFFT(POWERQUAD, 1, &input_value, &output_value);
        *output_data = output_value;

        return pushresults(cc, result);
    }

    // Cas matrice : appliquer IFFT complexe
    if (hd->type == s_matrix)
    {
        getmatrix(hd, &r, &c, &input_data);

        // Vérification des dimensions
        if (r != 1)
        {
            cc_error(cc, "row vector expected");
            return NULL;
        }

        // Allouer mémoire pour le résultat
        result = new_cmatrix(cc, 1, c, "");
        output_data = matrixof(result);

        // Appliquer la IFFT complexe
        PQ_TransformIFFT(POWERQUAD, c, input_data, output_data);

        return pushresults(cc, result);
    }

    // Type non supporté
    cc_error(cc, "unsupported data type");
    return NULL;
}


/****************************************************************
 *	Audio play
 ****************************************************************/
/* pcmfreq: get the CODEC sampling frequency
 *   fs=pcmfreq()
 *****/
header* mpcmfreq0 (Calc *cc, header *hd)
{
	return new_real(cc,pcm_get_freq(),"");
}

/* pcmfreq: set the CODEC sampling frequency, return the updated freq
 *   fs=pcmfreq()
 *****/
header* mpcmfreq (Calc *cc, header *hd)
{
	header *result;
	hd=getvalue(cc, hd);
	if (hd->type!=s_real) cc_error(cc,"real sample freq expected!");
	result=new_real(cc,pcm_set_freq(*realof(hd)),"");
	return pushresults(cc,result);
}

/* pcmvol: set the left and right output CODEC levels
 *   ok=pcmvol([left,right]) left and right output volume levels
 *****/
header* mpcmvol (Calc *cc, header *hd)
{
	header *result;
	real *m;

	hd=getvalue(cc, hd);

	if (hd->type!=s_real && !(hd->type==s_matrix && dimsof(hd)->r==1 && dimsof(hd)->c==2)) cc_error(cc,"bad parameter!");
	if (hd->type==s_real) {
		result=new_matrix(cc,1,2,"");
		m=matrixof(result); m[0]=*realof(hd);m[1]=*realof(hd);
	} else {
		result=hd;
		m=matrixof(hd);
	}
	if (m[0]<0.0 || m[0]>100.0 || m[1]<0.0 || m[1]>100.0) cc_error(cc,"volume is not in the range 0..100");
	result=new_real(cc,(real)pcm_vol(m),"");
	return pushresults(cc,result);
}

/* pcmplay: play samples in [1xn] or [2xn] vector or a file
 *   pcmplay(vector) | pcmplay(filename)
 *****/
header* mpcmplay (Calc *cc, header *hd)
{	header *result;
	real *m;
	int r,c;

	hd=getvalue(cc,hd);
	if (hd->type!=s_matrix && dimsof(hd)->r<1) cc_error(cc,"[1xn] or [2xn] real matrix expected!");
	getmatrix(hd,&r,&c,&m);

	result = new_real(cc,(real)pcm_play(m,r,c),"");

	return pushresults(cc,result);
}

/* mpcmrec: record n samples
 *  [2xn]=pcmrec(n)
 *****/
header* mpcmrec(Calc *cc, header *hd)
{
	header *result;
	hd=getvalue(cc,hd);
	if (hd->type!=s_real) cc_error(cc,"real value expected!");
	result=new_matrix(cc,2,*realof(hd),"");
	if (!pcm_rec(matrixof(result),*realof(hd))) cc_error(cc,"PCM Io error!");

	return pushresults(cc,result);
}

/* pcmloop: sample, modify, output the signal
 *   fs=pcmloop()
 *****/
header* mpcmloop (Calc *cc, header *hd)
{
	return new_real(cc,(real)pcm_loop(),"");
}

/* UPOLS
*	stack: filename -- 1d matrix
*/
header* mupols (Calc *cc, header *hd)
{
	header *result;
	real *m;
	int r,c;

	result = mreadmatrix(cc, hd);
	getmatrix (result, &r, &c, &m);
	if (r > 1 ) cc_error(cc, "only 1D matrix is supported");

	return new_real(cc, (real)UPOLS_start(m, r, c), "");

	// real *m;
	// int r,c;
	// getmatrix(hd,&r,&c,&m);
	// if (dimsof(hd)->c>1 && dimsof(hd)->r>1 ) {
	// 	cc_error(cc, "Input matrix must be 1D");
	// }
	// getmatrix(hd,&r,&c,&m);
	// return new_real(cc, (real)UPOLS_start(m, r, c), "");
}


