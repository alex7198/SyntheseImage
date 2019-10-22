/********************************************************/
/*                     cube.cpp                                                 */
/********************************************************/
/*                Affiche a l'ecran un cube en 3D                      */
/********************************************************/

/* inclusion des fichiers d'en-tete freeglut */

#ifdef __APPLE__
#include <GLUT/glut.h> /* Pour Mac OS X */
#else
#include <GL/freeglut.h>   /* Pour les autres systemes */
#endif
#include <cstdlib>
#include <math.h>
#include <iostream>
#include <jpeglib.h>

/*class Point*/
class Point{
    public :
	//coordonnées x, y et z du point
	double x;
	double y;
	double z;
	// couleur r, v et b du point
	float r;
	float g;
	float b;
};

char presse;
int anglex,angley,x,y,xold,yold;

//Variable pour la représentation paramétrique du cylindre

//n: pas du cylindre du cylindre.
//fCylT: tableau des points par face des "triangles" composant les deux faces circulaire du cylindre.
//fCylR: tableau des points par face des "rectangles" composant la face situées en les deux faces circulaire du cylindre.
//PCyl: tableau des coordonnées des points du cylindre.
int const n=50;
int fCylT[n*2+1][3];
int fCylR[n][4];
Point PCyl[2*n+2];
Point PCylTex[2*n+2];

GLuint texObject[2];

//Couleurs
GLfloat jaune[]={1,0.8,0,1.0};
GLfloat orange[]={1,0.5,0.1,1.0};
GLfloat marron[]={0.5,0.25,0,1.0};
GLfloat bleu[]={0.4,0.4,1,1.0};
GLfloat gris_clair[]={0.8,0.8,0.8,1.0};
GLfloat gris_fonce[]={0.2,0.2,0.2,1.0};


//Animations
float angleBrasG = 0, angleBrasD = 70;  //Angles entre les bras et le corps
int sens = 1, sensD = -1;               //Définit si les angles du bras gauche et du bras droit s'incrémentent ou diminuent

//Angles utiles pour le calcul de la position de la caméra
float theta = M_PI/2; //Initialisation à Pi/2
float phi = 0;

float distance = 10;    //Distance entre l'objet et la caméra
float marche=0;         //Angle des jambes qui permet de simuler la marche
int sensMarche=1;       //Indique si l'angle entre la jambe et le corps doit s'incrémenter (1) ou se décrémenter (-1)

//Au lancement de l'application, seule la lumière spéculaire est allumée
bool lumAmbiante = false;
bool lumDiffuse = false;
bool lumSpeculaire = true;

/* Prototype des fonctions */
void affichage();
void clavier(unsigned char touche,int x,int y);
void fleches(int touche, int x, int y);
void reshape(int x,int y);
void idle();
void mouse(int bouton,int etat,int x,int y);
void mousemotion(int x,int y);

//Fonctions de modélisation des primitives
void rempliCylindre(float r, float h, int n);
void dessinCylindre();
void cylindre(float r, float h, int n);
void cone(float rayon, int slices, float hauteur);

//Fonctions en rapport avec les textures
void loadJpegImage(char *fichier, int hauteur, int largeur, unsigned char * image);
void applicationTexture();
void parametreTexture(unsigned int hauteur, unsigned int largeur);
void applicationTextureTete();
void parametreTextureTete(unsigned int hauteur, unsigned int largeur);
void rempliCylindreTexture(float r, float h, int n);

//Fonctions en rapport avec la lumière
void lumiereAmbiante();
void lumiereDiffuse();
void lumiereSpeculaire();

//Fonctions de dessin du robot
void dessinPlot();
void dessinCorps();
void dessinTete();
void dessinHautCorps();
void dessinJambes();

//Dessin Tete
void dessinCheveux();
void dessinCrane();
void dessinVisage();
void dessinOeil(int cote);
void dessinBouche();
void dessinCou();

//Dessin Corps
void dessinBras(float RBras, float Lbras);
void dessinEpaule();
void dessinArriereBras(float Rbras, float Lbras);
void dessinArticulation(float RArti);
void dessinAvantBras(float Rbras, float Lbras);
void dessinMain();
void dessinPoignet();
void dessinDoigts(float lDoigt, float Ldoigt,float Hdoigt, float Pdoigt);
void dessinBuste(float lBuste, float Lbuste, float Hbuste, float Pbuste);
void dessinBassin(float lBassin,float LBassin,float HBassin,float PBassin);

//Dessin Jambes
void dessinJambe();
void dessinJambeHaut();
void dessinArticulationCylindre();
void dessinArticulationCube();
void dessinJambeBas();
void dessinGenou();
void dessinPied();

int main(int argc,char **argv)
{
  /* initialisation de glut et creation
     de la fenetre */
  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowPosition(200,200);
  glutInitWindowSize(500,500);
  glutCreateWindow("Lego");

  //Chargement texture
  parametreTexture(512,512);

  /* Initialisation d'OpenGL */
  glClearColor(0.0,0.0,0.0,0.0);
  glColor3f(1.0,1.0,1.0);
  glPointSize(2.0);
  glEnable(GL_DEPTH_TEST);

  /* enregistrement des fonctions de rappel */
  glutDisplayFunc(affichage);
  glutKeyboardFunc(clavier);
  glutSpecialFunc(fleches); //Gère les interactions de l'utilisateur avec les flèches
  glutReshapeFunc(reshape);
  glutMouseFunc(mouse);
  glutMotionFunc(mousemotion);

  /* Entree dans la boucle principale glut */
  glutMainLoop();
  return 0;
}


void affichage()
{
    GLfloat position_sourceO[]={0.0,2.0,5.0,1.0};

  /* effacement de l'image avec la couleur de fond */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glShadeModel(GL_SMOOTH);

    //Gestion de la distance par rapport à l'objet
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(-distance,distance,-distance+9.75/2,distance+9.75/2,-10,100);   //9.75 correspond à la taille approximative du robot

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //Lumière diffuse
    lumiereAmbiante();
    lumiereDiffuse();
    lumiereSpeculaire();

    glRotatef(angley,1.0,0.0,0.0);
    glRotatef(anglex,0.0,1.0,0.0);

    //Gestion de la caméra
    gluLookAt(cos(theta)*cos(phi),sin(phi),sin(theta)*cos(phi),0,0,0,0,1,0);

    glColor3f(1,0.8,0);     //La couleur par défaut est le jaune correspondant à la couleur de la peau du lego
    glutIdleFunc(idle);     //Animation

    //Dessin du robot
    dessinCorps();
	dessinPlot();

    //Dessin du repère
    //axe x en rouge
    glBegin(GL_LINES);
        glColor3f(1.0,0.0,0.0);
    	glVertex3f(0, 0,0.0);
    	glVertex3f(1, 0,0.0);
    glEnd();
    //axe des y en vert
    glBegin(GL_LINES);
    	glColor3f(0.0,1.0,0.0);
    	glVertex3f(0, 0,0.0);
    	glVertex3f(0, 1,0.0);
    glEnd();
    //axe des z en bleu
    glBegin(GL_LINES);
    	glColor3f(0.0,0.0,1.0);
    	glVertex3f(0, 0,0.0);
    	glVertex3f(0, 0,1.0);
    glEnd();

  glFlush();

  //On echange les buffers
  glutSwapBuffers();
}

void clavier(unsigned char touche,int x,int y)
{
  switch (touche)
    {
    case 'p': /* affichage du carre plein */
      glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
      glutPostRedisplay();
      break;
    case 'f': /* affichage en mode fil de fer */
      glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
      glutPostRedisplay();
      break;
    case 's' : /* Affichage en mode sommets seuls */
      glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
      glutPostRedisplay();
      break;
    case 'd':
      glEnable(GL_DEPTH_TEST);
      glutPostRedisplay();
      break;
    case 'D':
      glDisable(GL_DEPTH_TEST);
      glutPostRedisplay();
      break;
    case 'Z' :  /*Avance la camera vers le robot*/
        if(distance+0.5<100) distance+=0.5;
        glutPostRedisplay();
    break;
    case 'z' :  /*Recule la camera du robot*/
        if(distance-0.5>=0) distance-=0.5;
        glutPostRedisplay();
    break;
    case 'm' :  /*Provoque l'animation de marche*/
        if(marche==70) sensMarche = -1;
        if(marche==-70) sensMarche = 1;
        marche+=sensMarche*5;
        glutPostRedisplay();
    break;
    case 'w' :  /*Allume ou éteint la lumière ambiante*/
        lumAmbiante = !lumAmbiante;
        glutPostRedisplay();
    break;
    case 'x' :  /*Allume ou éteint la lumière diffuse*/
        lumDiffuse = !lumDiffuse;
        glutPostRedisplay();
    break;
    case 'c' :  /*Allume ou éteint la lumière spéculaire*/
        lumSpeculaire = !lumSpeculaire;
        glutPostRedisplay();
    break;
    case 'q' : /*la touche 'q' permet de quitter le programme */
      exit(0);
    }
}

//Gère les cas où l'utilisateur utilise les flèches du clavier
void fleches(int touche, int x, int y){
    switch(touche){
        case GLUT_KEY_UP:   /*Fait tourner la caméra autour du robot par le haut*/
            phi += 0.1;
            if(phi>2*M_PI) phi = 0;
            glutPostRedisplay();
        break;
        case GLUT_KEY_DOWN: /*Fait tourner la caméra autour du robot par le bas*/
            phi -= 0.1;
            if(phi<0) phi = 2*M_PI;
            glutPostRedisplay();
        break;
        case GLUT_KEY_LEFT: /*Fait tourner la caméra autour du robot par la gauche*/
            theta-=0.1;
            if(theta<0) theta = 2*M_PI;
            glutPostRedisplay();
        break;
        case GLUT_KEY_RIGHT:    /*Fait tourner la caméra autour du robot par la droite*/
            theta+=0.1;
            if(theta>2*M_PI) theta = 0;
            glutPostRedisplay();
        break;
    }
}

void reshape(int x,int y)
{
  if (x<y)
    glViewport(0,(y-x)/2,x,x);
  else
    glViewport((x-y)/2,0,y,y);
}

void mouse(int button, int state,int x,int y)
{
  /* si on appuie sur le bouton gauche */
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
  {
    presse = 1; /* le booleen presse passe a 1 (vrai) */
    xold = x; /* on sauvegarde la position de la souris */
    yold=y;
  }
  /* si on relache le bouton gauche */
  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    presse=0; /* le booleen presse passe a 0 (faux) */
}

void mousemotion(int x,int y)
  {
    if (presse) /* si le bouton gauche est presse */
    {
      /* on modifie les angles de rotation de l'objet
	 en fonction de la position actuelle de la souris et de la derniere
	 position sauvegardee */
      anglex=anglex+(x-xold);
      angley=angley+(y-yold);
      glutPostRedisplay(); /* on demande un rafraichissement de l'affichage */
    }

    xold=x; /* sauvegarde des valeurs courante de le position de la souris */
    yold=y;
  }

//Gère l'animation par défaut (mouvemnt des bras : Le bras gauche balance légèrement, le bras droit fait coucou)
void idle(){
    /*Dès que l'angle du bras atteint sa valeur maximale, le sens d'incrémentation s'inverse et le bras part dans l'autre sens*/
    if(angleBrasG==60) sens = -1;
    if(angleBrasG==0) sens = 1;

    if(floor(angleBrasD)==70) sensD = -1;
    if(floor(angleBrasD)==60) sensD = 1;

    angleBrasG+=sens*2.0;
    angleBrasD+=sensD*0.1;


    glutPostRedisplay( );
}

  /*----------------------------------------------------------------------------
    Fonctions en rapport avec la modélisation des primitives
    ----------------------------------------------------------------------------*/

    //Fonction remplissant les tableaux de coordonnées des points du cylindre et les tableaux des points par face du cylindre.
    void rempliCylindre(float r, float h, int n){
        //Points du cylindre de la première face circulaire du cylindre
        for(int i=0; i<n; i++){
            PCyl[i] = Point{r*cos(2*i*M_PI/n),r*sin(2*i*M_PI/n),h};
        }
        //Point du centre de la première face circulaire
        PCyl[n] = Point{0,0,h};

        //Points du cylindre de la deuxième face circulaire du cylindre.
        //(2*M_PI*(i-1))/n --> le i-1 se justifie car on stock le centre de la première face en PCyl[n] (i-1 pour éviter un décalage).
        for(int i=n+1; i<2*n+1; i++){
            PCyl[i] = Point{r*cos((2*M_PI*(i-1))/n),r*sin((2*M_PI*(i-1))/n),0};
        }
        //Point du centre de la seconde face circulaire.
        PCyl[2*n+1] = Point{0,0,0};

        //Faces du cylindre

        //Remplissage du tableau de points par face pour les faces rectangulaire.
        for(int i=0; i<n; i++){
            fCylR[i][0]=i;
            fCylR[i][1]=i+1+n;
            fCylR[i][2]=((i+1)%n)+n+1;
            fCylR[i][3]=(i+1)%n;
        }

        //Remplissage du tableau de points par face pour les deux faces circulaires.
        for(int i=0; i<n; i++){
            fCylT[i][0]=i;
            fCylT[i][1]=(i+1)%n;
            fCylT[i][2]=n;
        }

        for(int i=0; i<n; i++){
            fCylT[i+n][0]=i+n+1;
            fCylT[i+n][1]=2*n+1;
            fCylT[i+n][2]=n+1+((i+1)%n);
        }
    }

    //Dessin du cylindre
    void dessinCylindre()
    {
        int l = 0;
        //Dessin des faces circulaires
        for (l =0;l<n*2+1;l++)
        {
        glBegin(GL_POLYGON);
          for (int j=0;j<3;j++){
              glVertex3f(PCyl[fCylT[l][j]].x,PCyl[fCylT[l][j]].y,PCyl[fCylT[l][j]].z);
          }
        glEnd();
        }
        //Dessin des faces rectangulaires
        for (l=0;l<n;l++)
        {
          for (int j=0;j<4;j++){
          glBegin(GL_POLYGON);
              glVertex3f(PCyl[fCylR[l][j]].x,PCyl[fCylR[l][j]].y,PCyl[fCylR[l][j]].z);
          }
            glEnd();
        }
    }

    void cylindre(float r, float h, int n)
    {
        //Remplissage des tableaux de stockage de coordonnées et de points par face
        rempliCylindre(r,h,n);
        //Remplissage des tableaux de stockage de coordonnées et de points par face pour la texture plaquée.
        rempliCylindreTexture(r,h,n);
        //On enroule la texture autour du cylindre
        applicationTextureTete();
        //On dessine le cylindre
        dessinCylindre();
    }

    void rempliCylindreTexture(float r, float h, int n)
    {
        //On augmente le rayon d'un tout petit peu pour éviter le clignotement entre la texture et la couleur du cylindre
        r+=0.0001;
        //Points du cylindre de la première face circulaire du cylindre
        for(int i=0; i<n; i++){
            PCylTex[i] = Point{r*cos(2*i*M_PI/n),r*sin(2*i*M_PI/n),h};
        }
        //Point du centre de la première face circulaire
        PCylTex[n] = Point{0,0,h};

        //Points du cylindre de la deuxième face circulaire du cylindre.
        //(2*M_PI*(i-1))/n --> le i-1 se justifie car on stock le centre de la première face en PCyl[n] (i-1 pour éviter un décalage).
        for(int i=n+1; i<2*n+1; i++){
            PCylTex[i] = Point{r*cos((2*M_PI*(i-1))/n),r*sin((2*M_PI*(i-1))/n),0};
        }
        //Point du centre de la seconde face circulaire.
        PCylTex[2*n+1] = Point{0,0,0};
    }

	void cone(float rayon, int slices, float hauteur){
	    float v1x, v1y, v1z, v2x, v2y, v2z, pvx, pvy, pvz;

		//Tableau pour stocker les sommets du cone et leur couleur
		Point pCone[slices+2];

		//Tableau pour stocker les indices des sommets par face pour le cone
		int fCone[slices*2][3];

		//Création des points
		//Points sur le cercle
		for(int i=0; i<slices; i++){
			pCone[i].x = rayon*cos(2*i*M_PI/slices);
			pCone[i].y = 0;
			pCone[i].z = rayon*sin(2*i*M_PI/slices);
		}
		//Centre du cercle
		pCone[slices].x = 0;
		pCone[slices].y = 0;
		pCone[slices].z = 0;
		//Sommet du cone
		pCone[slices+1].x = 0;
		pCone[slices+1].y = hauteur;
		pCone[slices+1].z = 0;

		//Création des faces
		for(int i=0; i<slices; i++){
			fCone[i][0] = i;
			fCone[i][1] = (i+1)%slices;
			fCone[i][2] = slices;
			fCone[i+slices][0] = i;
			fCone[i+slices][1] = (i+1)%slices;
			fCone[i+slices][2] = slices+1;
		}

		// Dessin du cone
		for (int i=0;i<slices*2;i++)
		{
            glBegin(GL_POLYGON);

            //Calcul des coordonnées de deux vecteurs du plan
            v1x=pCone[fCone[i][0]].x-pCone[fCone[i][1]].x;
            v1y=pCone[fCone[i][0]].y-pCone[fCone[i][1]].y;
            v1z=pCone[fCone[i][0]].z-pCone[fCone[i][1]].z;
            v2x=pCone[fCone[i][1]].x-pCone[fCone[i][2]].x;
            v2y=pCone[fCone[i][1]].y-pCone[fCone[i][2]].y;
            v2z=pCone[fCone[i][1]].z-pCone[fCone[i][2]].z;
            //Calcul du produit vectoriel afin de trouver un vecteur normal au plan(les deux vecteurs calculés précédemment ne doivent pas être colinéaires)
            pvx=v1y*v2z-v1z*v2y;
            pvy=v1z*v2x-v1x*v2z;
            pvz=v1x*v2y-v1y*v2x;
			//Normale pour la lumière
            glNormal3d(-pvx,-pvy,-pvz);

            for (int j=0;j<3;j++){
              glVertex3f(pCone[fCone[i][j]].x,pCone[fCone[i][j]].y,pCone[fCone[i][j]].z);
            }
            glEnd();
		}
	}

    /*----------------------------------------------------------------------------
    Fonctions en rapport avec les textures
    ----------------------------------------------------------------------------*/

    void loadJpegImage(char *fichier, int hauteur, int largeur, unsigned char * image)
    {
      struct jpeg_decompress_struct cinfo;
      struct jpeg_error_mgr jerr;
      FILE *file;
      unsigned char *ligne;
      cinfo.err = jpeg_std_error(&jerr);
      jpeg_create_decompress(&cinfo);
    #ifdef __WIN32
      if (fopen_s(&file,fichier,"rb") != 0)
        {
          fprintf(stderr,"Erreur : impossible d'ouvrir le fichier texture.jpg\n");
          exit(1);
        }
    #elif __GNUC__
      if ((file = fopen(fichier,"rb")) == 0)
        {
          fprintf(stderr,"Erreur : impossible d'ouvrir le fichier texture.jpg\n");
          exit(1);
        }
    #endif
      jpeg_stdio_src(&cinfo, file);
      jpeg_read_header(&cinfo, TRUE);

      if ((cinfo.image_width!=largeur)||(cinfo.image_height!=hauteur)) {
        fprintf(stdout,"Erreur : l'image doit etre de taille 256x256\n");
        exit(1);
      }
      if (cinfo.jpeg_color_space==JCS_GRAYSCALE) {
        fprintf(stdout,"Erreur : l'image doit etre de type RGB\n");
        exit(1);
      }

      jpeg_start_decompress(&cinfo);
      ligne=image;
      while (cinfo.output_scanline<cinfo.output_height)
        {
          ligne=image+3*largeur*cinfo.output_scanline;
          jpeg_read_scanlines(&cinfo,&ligne,1);
        }
      jpeg_finish_decompress(&cinfo);
      jpeg_destroy_decompress(&cinfo);
    }


    void parametreTexture(unsigned int hauteur, unsigned int largeur)
    {
        //Génération d'ID pour les textures
        glGenTextures(2, texObject);
        //Cravate
        unsigned char imgCravate[512*512*3];
        //On lie la texture au premier element du tableau de texture pour pouvoir la réutiliser facilement plus tard
        glBindTexture(GL_TEXTURE_2D,texObject[0]);
        //Chargement de l'image dans le tableau
        loadJpegImage("./textureBuste.jpg",512,512,imgCravate);
        /* Parametrage du placage de textures */
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
        //Création de la texture à partir de l'imae
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,512,512,0,
        GL_RGB,GL_UNSIGNED_BYTE,imgCravate);
        glBindTexture(GL_TEXTURE_2D,0);

        //Visage
        unsigned char imgTete[512*512*3];
        //On lie la texture au deuxième element du tableau de texture pour pouvoir la réutiliser facilement plus tard
        glBindTexture(GL_TEXTURE_2D,texObject[1]);
        //Chargement de l'image dans le tableau
        loadJpegImage("./textureVisage.jpg",512,512,imgTete);
        //Parametrage du placage de textures
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
        //Création de la texture
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,512,512,0,
        GL_RGB,GL_UNSIGNED_BYTE,imgTete);

    }

    void applicationTexture()
    {
        //Variables stockant les coodonnées de deux vecteurs du plan correspondant à la texture et les coordonnées
        //->du produit vectoriel de ces deux vecteurs
		float v1x,v1y,v1z,v2x,v2y,v2z,pvx,pvy,pvz;
		//Calcul des coordonnées des deux vecteurs
		v1x=(-1.15)-1.15;
		v1y=-1.5-1.5;
		v1z=0.455-0.455;
		v2x=1.15-(-1.15);
		v2y=1.5-1.5;
		v2z=0.455-0.455;
		//Calcul du produit vectoriel
		pvx=v1y*v2z-v1z*v2y;
		pvy=v1z*v2x-v1x*v2z;
		pvz=v1x*v2y-v1y*v2x;
		//On rapelle la texture à plaquer
		glBindTexture(GL_TEXTURE_2D,texObject[0]);
		glEnable(GL_TEXTURE_2D);
            //On spécifie comment la couleur de la texture doit réagir face à la lumière
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);
            glBegin(GL_POLYGON);
            //On applique un vecteur normal pour la texture afin que les textures aient un comportement cohérant
            //->face aux lumières.
            glNormal3d(pvx,pvy,pvz);
            //On met en place la texture
            //Pour le glVertex, voir mise à l'échelle du cube/2
            glTexCoord2f(0.0,0.0);   glVertex3f(-1.15, 1.5, 0.455);
            glTexCoord2f(0.0,1.0);   glVertex3f(-1.15,-1.5, 0.455);
            glTexCoord2f(1.0,1.0);   glVertex3f( 1.15,-1.5, 0.455);
            glTexCoord2f(1.0,0.0);   glVertex3f( 1.15, 1.5, 0.455);
		  glEnd();
		  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
		  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
		glDisable(GL_TEXTURE_2D);
    }

    void applicationTextureTete()
    {
        //Variables stockant les coodonnées de deux vecteurs du plan correspondant à la texture et les coordonnées
        //->du produit vectoriel de ces deux vecteurs
		float v1x,v1y,v1z,v2x,v2y,v2z,pvx,pvy,pvz;
		//On appelle la texture à utiliser
		glBindTexture(GL_TEXTURE_2D,texObject[1]);
		glEnable(GL_TEXTURE_2D);
		//Pour toutes les faces du cylindre...
		for(int i=0;i<n;i++)
		{
			int j=0;
			float a=i,b=i;
			a=a/n;
			//Calcul des coordonnées de deux vecteurs du plan
			v1x=PCylTex[fCylR[i][j]].x-PCylTex[fCylR[i][j+1]].x;
			v1y=PCylTex[fCylR[i][j]].y-PCylTex[fCylR[i][j+1]].y;
			v1z=PCylTex[fCylR[i][j]].z-PCylTex[fCylR[i][j+1]].z;
			v2x=PCylTex[fCylR[i][j+1]].x-PCylTex[fCylR[i][j+3]].x;
			v2y=PCylTex[fCylR[i][j+1]].y-PCylTex[fCylR[i][j+3]].y;
			v2z=PCylTex[fCylR[i][j+1]].z-PCylTex[fCylR[i][j+3]].z;
			//Calcul du produit vectoriel afin de trouver un vecteur normal au plan(les deux vecteurs calculés précédemment ne doivent pas être colinéaires)
			pvx=v1y*v2z-v1z*v2y;
			pvy=v1z*v2x-v1x*v2z;
			pvz=v1x*v2y-v1y*v2x;
			//4 sommets du rectangle
			glBegin(GL_POLYGON);
			//On définit le vecteur normal pour le plan en cours pour appliquer la lumière correctement.
			glNormal3d(pvx,pvy,pvz);
			//On applique la bonne partie de texture, au bon endroit
			glTexCoord2f(a,0.0);
			glVertex3f(PCylTex[fCylR[i][j+1]].x,PCylTex[fCylR[i][j+1]].y,PCylTex[fCylR[i][j+1]].z);
			glTexCoord2f(a,1.0);
			glVertex3f(PCylTex[fCylR[i][j]].x,PCylTex[fCylR[i][j]].y,PCylTex[fCylR[i][j]].z);
			a=(b+1)/n;
			glTexCoord2f(a,1.0);
			glVertex3f(PCylTex[fCylR[i][j+3]].x,PCylTex[fCylR[i][j+3]].y,PCylTex[fCylR[i][j+3]].z);
			glTexCoord2f(a,0.0);
			glVertex3f(PCylTex[fCylR[i][j+2]].x,PCylTex[fCylR[i][j+2]].y,PCylTex[fCylR[i][j+2]].z);
			glEnd();
		}
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
		glDisable(GL_TEXTURE_2D);
    }

  /*----------------------------------------------------------------------------
    Fonctions en rapport avec la lumière
    ----------------------------------------------------------------------------*/

    void lumiereDiffuse()
    {
        //Activation de l'éclairage
        glEnable(GL_LIGHTING);
        //On définit la position de la source lumineuse
        GLfloat position_sourceO[]={0,8,5,1};
        //Lumière diffuse
        glLightfv(GL_LIGHT1,GL_POSITION,position_sourceO);
        //lumière diffuse blanche
        GLfloat dif_O[]={1,1,1,0};
        glLightfv(GL_LIGHT1,GL_DIFFUSE,dif_O);
        if(lumDiffuse) glEnable(GL_LIGHT1);
        else glDisable(GL_LIGHT1);
    }

    void lumiereAmbiante()
    {
        //Activation de l'éclairage
        glEnable(GL_LIGHTING);
        //On définit la position de la source lumineuse
        GLfloat position_sourceO[]={0,8,5,1};
        //Lumière ambiante
        glLightfv(GL_LIGHT3,GL_POSITION,position_sourceO);
        //lumière ambiante
        GLfloat dif_O[]={0.7,0.7,0.7,0};
        glLightfv(GL_LIGHT3,GL_AMBIENT,dif_O);
        if(lumAmbiante) glEnable(GL_LIGHT3);
        else glDisable(GL_LIGHT3);
    }

    void lumiereSpeculaire()
    {
        //Activation de l'éclairage
        glEnable(GL_LIGHTING);
        //On définit la position
        GLfloat position_sourceO[]={0,5,15,1};
        //Lumière spéculaire blanche
        GLfloat dif_O[]={1,1,1,0};
        //Direction de la lumière
        GLfloat direction_source0[]={0,0,-1};
        //On applique la position définit avant
        glLightfv(GL_LIGHT2,GL_POSITION,position_sourceO);
        //On réduit l'angle d'ouverture de la source lumineuse
        glLightf(GL_LIGHT2,GL_SPOT_CUTOFF,40.0);
        //On applique la direction
        glLightfv(GL_LIGHT2,GL_SPOT_DIRECTION,direction_source0);
        //On définit la lumière
        glLightfv(GL_LIGHT2,GL_SPECULAR,dif_O);
        //Atténuation en fonction de l'éloignement
        glLightf(GL_LIGHT2, GL_CONSTANT_ATTENUATION, 1);
        //On définit l'intensité de la lumière
        glLightf(GL_LIGHT2, GL_SPOT_EXPONENT, 2.0);
        if(lumSpeculaire) glEnable(GL_LIGHT2);
        else glDisable(GL_LIGHT2);
    }

  /*----------------------------------------------------------------------------
    Fonctions de dessin du Robot
    ----------------------------------------------------------------------------*/

//Cette fonction dessine le plot à côté du personnage
void dessinPlot(){
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,orange);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,orange);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,orange);

	glTranslatef(-5,0.2,0);

    //Dessine la base rectangulaire
    glPushMatrix();
        glScalef(1,0.2,1);
        glutSolidCube(2);
    glPopMatrix();

    //Dessine le cone
    glPushMatrix();
        glTranslatef(0,0.2,0);
        cone(0.9,40,2.5);

		//Dessine la bande grise
		glPushMatrix();
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,gris_clair);
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,gris_clair);
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,gris_clair);
			glTranslatef(0,1.1,0);
			glRotatef(90,1,0,0);
            glutSolidCylinder(0.6,0.6,20,20);
		glPopMatrix();
    glPopMatrix();

	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
}

//Cette fonction dessine le robot en entier
void dessinCorps(){
    //Dessine les jambes
    glPushMatrix();
        glScalef(1,1,0.8);
        dessinJambes();
    glPopMatrix();

    //Dessine le haut du corps (bras + tronc)
    glPushMatrix();
        glTranslatef(0,3.15,0);
        glScalef(1,1,1.75/0.8);
        dessinHautCorps();
    glPopMatrix();

    //Dessine la tête
    glPushMatrix();
        glTranslatef(0,6.55,0);
        dessinTete();
    glPopMatrix();
}

//************************** Dessin de la tête

//Dessine la tête du robot
void dessinTete(){
    glPushMatrix();
        //Dessin du cou
        dessinCou();

        //Dessin du crane
        glTranslatef(0,2,0);
        dessinCrane();

        //Dessin du visage (yeux et bouche)
        glTranslatef(0,-1,0);

        //Dessin des cheveux (casque du lego)
        glTranslatef(0,0.5,0);
        dessinCheveux();
    glPopMatrix();
}

//Dessin des cheveux
void dessinCheveux(){
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,marron);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,marron);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,marron);

    glPushMatrix();
        glTranslatef(0,0,-0.3);
        glScalef(1,1,0.8);
        glutSolidSphere(1.3,20,20);
    glPopMatrix();
        glTranslatef(0,0.5,0);
        glScalef(1.1,0.8,1.3);
        glutSolidSphere(1,20,20);
    glPopMatrix();

    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
}

//Dessine le crane
void dessinCrane(){
    glPushMatrix();
        glRotatef(90,1,0,0);
        cylindre(1,2,n);
    glPopMatrix();
}

//Dessine le cou
void dessinCou(){
    glPushMatrix();
        //Dessin du cou
        glPushMatrix();
            glRotatef(90,1,0,0);
            glutSolidCylinder(0.6,0.2,20,20);
        glPopMatrix();

        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,gris_clair); //Couleur grise claire
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,gris_clair);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,gris_clair);

        //Dessin du col
        glPushMatrix();
            glTranslatef(0,-0.1,0);
            glRotatef(90,1,0,0);
            glutSolidCylinder(0.7,0.1,20,20);
        glPopMatrix();

        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune); //Couleur jaune
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
    glPopMatrix();
}

//************************** Dessin du haut du corps

//Dessine le haut du corps
void dessinHautCorps()
{
    //Dessin du tronc/buste
    dessinBuste(1,2.3,3,0.9);

    //Dessin du bras gauche
    glPushMatrix();
        glTranslatef(2.25/2,2.6,0);
        glRotatef(angleBrasG,0,0,1);
        glScalef(1,1.5,1);
        dessinBras(0.5,1.5);
    glPopMatrix();

    //Dessin du bras droit
    glPushMatrix();
        glTranslatef(-2.25/2,2.6,0);
        glRotatef(angleBrasD,0,0,1);
        glRotatef(180,0,1,0);
        glScalef(1,1.5,1);
        dessinBras(0.5,1.5);
    glPopMatrix();

    //Dessin du bassin
    dessinBassin(1,2.25,0.4,0.8);
}

//Dessine le buste/le tronc
void dessinBuste(float lBuste, float Lbuste, float Hbuste, float Pbuste)
{
    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,orange);       //Couleur orange
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,orange);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,orange);
        glTranslatef(0,1.7,0);

        applicationTexture();

        //Tronc
        glPushMatrix();
            glScalef(Lbuste,Hbuste,Pbuste);
            glutSolidCube(lBuste);
        glPopMatrix();

        //Bandes grises horizontales
        glPushMatrix();
            glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,gris_clair);
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,gris_clair);
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,gris_clair);
            glTranslatef(0,-1,-0.025);
            glScalef(Lbuste+0.05,0.35,Pbuste+0.025);
            glutSolidCube(1);
        glPopMatrix();

        glPushMatrix();
            glTranslatef(0,1,-0.025);
            glScalef(Lbuste+0.05,0.35,Pbuste+0.025);
            glutSolidCube(1);
        glPopMatrix();

        //Bandes grises verticales
        glPushMatrix();
            glTranslatef(0.8,0,-0.025);
            glScalef(0.25,Hbuste+0.05,Pbuste+0.025);
            glutSolidCube(1);
        glPopMatrix();

        glPushMatrix();
            glTranslatef(-0.8,0,-0.025);
            glScalef(0.25,Hbuste+0.05,Pbuste+0.025);
            glutSolidCube(1);
        glPopMatrix();

        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);     //Couleur jaune
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
    glPopMatrix();
}

//Dessine le bras du robot
void dessinBras(float RBras, float Lbras)
{
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,bleu);      //Couleur bleue
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,bleu);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,bleu);

    //Dessin de l'épaule
    dessinEpaule();

    //Dessin de l'avant bras
    dessinArriereBras(0.25,1);

    //Dessin du coude
    dessinArticulation(0.25);

    //Dessin de l'avant bras
    dessinAvantBras(0.25, 1);

    //Dessin de la main
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);     //Couleur jaune
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
    dessinMain();
}

//Dessine l'épaule
void dessinEpaule(){
    glPushMatrix();
        glutSolidSphere(0.3,20,20);
    glPopMatrix();
}

//Dessine l'arriere-bras (entre le coude et l'épaule)
void dessinArriereBras(float Rbras, float Lbras)
{
    glPushMatrix();
        glRotatef(90,0,1,0);
        glutSolidCylinder(Rbras,Lbras,20,20);
    glPopMatrix();
}

//Dessine le coude
void dessinArticulation(float RArti)
{
    glPushMatrix();
        glTranslatef(1.1,0,0);
        glutSolidSphere(RArti,20,20);
    glPopMatrix();
}

//Dessine l'avant-bras
void dessinAvantBras(float Rbras, float Lbras)
{
    glPushMatrix();
        glTranslatef(1.2,0,0);
        glRotatef(90,0,1,0);
        glutSolidCylinder(Rbras,Lbras,20,20);
    glPopMatrix();
}

//Dessine la main
void dessinMain(){
    glPushMatrix();
        glTranslatef(2.1+0.35/2,0,0);

        //Dessin du poignet
        glPushMatrix();
            glTranslatef(-0.35/2,0,0);
            dessinPoignet();
        glPopMatrix();

        //Dessin de la base sur laquelle se trouve les doigts
        glPushMatrix();
            glTranslatef(0.25,0,0);
            glScalef(0.2,0.6,0.35);
            glutSolidCube(1);
        glPopMatrix();

        //Dessin des doigts
        glPushMatrix();
        glTranslatef(0.15,0,0);
            dessinDoigts(1,0.45,0.1,0.3);
        glPopMatrix();

    glPopMatrix();
}

//Dessine le poignet
void dessinPoignet(){
    glPushMatrix();
        glRotatef(90,0,1,0);
        glutSolidCylinder(0.2,0.35,10,10);
    glPopMatrix();
}

//Dessine les doigts
void dessinDoigts(float lDoigt, float Ldoigt,float Hdoigt, float Pdoigt)
{
    //Doigt gauche
    glPushMatrix();
        glTranslatef(0.25+0.35/2,0.25,0);
        glScalef(Ldoigt,Hdoigt,Pdoigt);
        glutSolidCube(lDoigt);
    glPopMatrix();

    //Doigt droit
   glPushMatrix();
        glTranslatef(0.25+0.35/2,-0.25,0);
        glScalef(Ldoigt,Hdoigt,Pdoigt);
        glutSolidCube(lDoigt);
    glPopMatrix();
}

//Dessine le bassin
void dessinBassin(float lBassin,float LBassin,float HBassin,float PBassin)
{
    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,gris_fonce);     //Couleur gris très foncé
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,gris_fonce);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,gris_fonce);

        //Dessine la ceinture
        glPushMatrix();
            glScalef(LBassin,HBassin,PBassin);
            glutSolidCube(lBassin);
        glPopMatrix();

        //Dessine la boucle de la ceinture
        glPushMatrix();
            glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,gris_clair);     //Couleur gris clair
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,gris_clair);
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,gris_clair);
            glTranslatef(0,0,0.05);
            glScalef(0.4,0.4,0.8);
            glutSolidCube(1);
        glPopMatrix();

        //Dessin de la première accroche
        glPushMatrix();
            glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,orange);         //Couleur orange
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,orange);
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,orange);
            glTranslatef(-0.7,0,0);
            glScalef(0.2,0.4,0.85);
            glutSolidCube(1);
        glPopMatrix();

        //Dessin de la deuxième accroche
        glPushMatrix();
            glTranslatef(0.7,0,0);
            glScalef(0.2,0.4,0.85);
            glutSolidCube(1);
        glPopMatrix();

        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);     //Couleur jaune
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
    glPopMatrix();
}

//************************** Dessin du bas du corps

//Dessine les deux jambes
void dessinJambes(){
    //Jambe droite
    glPushMatrix();
        //Animation marche
        glTranslatef(0,3,0);
        glRotatef(marche,1,0,0);
        glTranslatef(0,-3,0);

        //Dessin
        glTranslatef((0.25/2)+0.5,0.35,0);
        dessinJambe();
    glPopMatrix();

    //Jambe gauche
    glPushMatrix();
        //Animation marche
        glTranslatef(0,3,0);
        glRotatef(-marche,1,0,0);
        glTranslatef(0,-3,0);

        //Dessin
        glTranslatef(-(0.25/2)-0.5,0.35,0);
        dessinJambe();
    glPopMatrix();
}

//Dessine une seule jambe
void dessinJambe(){
    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,orange);   //Couleur orange
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,orange);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,orange);

    //Dessin de la partie de la jambe au dessus du genou
    glPushMatrix();
        glTranslatef(0,2.2,-0.25/2);
        dessinJambeHaut();
    glPopMatrix();

    //Dessin de la partie de la jambe en dessous du genou
    glPushMatrix();
        glTranslatef(0,1.1,-0.25/2);
        dessinJambeBas();
    glPopMatrix();

    //Dessin du pied
    glPushMatrix();
        dessinPied();
    glPopMatrix();

    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);     //Couleur jaune
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
}

//Dessine le haut de la jambe (au dessus du genou)
//La forme est un peu spéciale, fusion entre un cylindre et un parallepipede rectangle
void dessinJambeHaut(){
    //Dessin de la partie cubique
    dessinArticulationCube();

    //Dessin et fusion avec la partie cylindrique
    glPushMatrix();
        glTranslatef(-0.5,0,1.75/2);
        dessinArticulationCylindre();
    glPopMatrix();
}

//Dessin de la partie cubique du haut de la jambe
void dessinArticulationCube(){
    glPushMatrix();
        glScalef(1,0.8,1.75);
        glutSolidCube(1);
    glPopMatrix();
}

//Dessin de la partie cylindrique du haut de la jambe
void dessinArticulationCylindre(){
    glPushMatrix();
        glRotatef(90,0,1,0);
        glutSolidCylinder(0.4,1,20,20);
    glPopMatrix();
}

//Dessin du bas de la jambe (genou et en dessous)
void dessinJambeBas(){
    glPushMatrix();
        //Dessin du tibia
        glScalef(1,1.5,1.75);
        glutSolidCube(1);

        //Dessin du genou
        glPushMatrix();
            dessinGenou();
        glPopMatrix();
    glPopMatrix();
}

//Dessin du genou
void dessinGenou(){
    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,gris_clair);     //Couleur grise
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,gris_clair);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,gris_clair);
        glScalef(1.1,0.5,1.1);
        glutSolidCube(1);
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);       //COuleur jaune
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
    glPopMatrix();
}

//Dessin du pied
void dessinPied(){
    glPushMatrix();
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,orange);     //Couleur orange
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,orange);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,orange);
        glScalef(1,0.7,2);
        glutSolidCube(1);
        glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,jaune);     //Couleur jaune
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,jaune);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,jaune);
    glPopMatrix();
}
