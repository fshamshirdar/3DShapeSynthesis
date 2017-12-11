#include <string.h>
#include <GL/glui.h>
#include "smf_parser.h"
#include "data.h"
#include "mix_match.h"
#include "control_points/eight_points.h"
#include "control_points/closest_connecting_points.h"
#include "control_points/hull_grid_points.h"
#include "control_points/box_intersection_points.h"
#include "mixers/knn_weighted_interpolation.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

float xy_aspect;
int   last_x, last_y;
float rotationX = 0.0, rotationY = 0.0;

/** These are the live variables passed into GLUI ***/
int   wireframe = 0;
int   obj_type = 1;
int   segments = 8;
int   segments2 = 8;
int   light0_enabled = 1;
int   light1_enabled = 1;
float light0_intensity = 1.0;
float light1_intensity = .4;
int   main_window;
float scale = 1.0;
int   show_sphere=1;
int   show_torus=1;
int   show_axes = 1;
int   show_text = 1;
float view_rotate[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
float obj_pos[] = { 0.0, 0.0, 0.0 };
const char *display_options[] = { "Flat shaded", "Smooth shaded", "Wireframe", "Shaded with mesh ..." };
int   curr_string = 0;

/** Pointers to the windows and some of the controls we'll create **/
GLUI *glui;
GLUI_FileBrowser *file_browser;

GLUI_Spinner    *light0_spinner, *light1_spinner;
GLUI_RadioGroup *radio;
GLUI_RadioGroup *display_radio;
GLUI_Listbox	*display_list;
GLUI_Panel      *display_panel;

SMFParser	*parser;
Data		*data = NULL;
Data		*data1 = NULL;
Data		*data2 = NULL;
MixMatch	*mixer = NULL;
std::vector<ControlPointsMiner::ControlPoint*> controlPoints;

/********** User IDs for callbacks ********/
#define OPEN_ID              100
#define CLOSE_ID             101
#define LIGHT0_ENABLED_ID    200
#define LIGHT1_ENABLED_ID    201
#define LIGHT0_INTENSITY_ID  250
#define LIGHT1_INTENSITY_ID  260
#define ENABLE_ID            300
#define DISABLE_ID           301
#define SHOW_ID              302
#define HIDE_ID              303
/********** Miscellaneous global variables **********/

GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
GLfloat light0_diffuse[] =  {.6f, .6f, 1.0f, 1.0f};
GLfloat light0_position[] = {.5f, .5f, 1.0f, 0.0f};

GLfloat light1_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
GLfloat light1_diffuse[] =  {.9f, .6f, 0.0f, 1.0f};
GLfloat light1_position[] = {-1.0f, -1.0f, 1.0f, 0.0f};

GLfloat lights_rotation[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

/**************************************** control_callback() *******************/
/* GLUI control callback                                                 */
void display_option_callback(int control)
{
}

void control_callback(int control)
{
  if (control == OPEN_ID) {
    std::string filename = "";
    filename = file_browser->get_file();
    data = parser->load(filename);
    // parser->simplify(50);
    glutPostRedisplay();
  }
  else if (control == LIGHT0_ENABLED_ID) {
    if ( light0_enabled ) {
      glEnable( GL_LIGHT0 );
      light0_spinner->enable();
    }
    else {
      glDisable( GL_LIGHT0 ); 
      light0_spinner->disable();
    }
  }
  else if ( control == LIGHT1_ENABLED_ID ) {
    if ( light1_enabled ) {
      glEnable( GL_LIGHT1 );
      light1_spinner->enable();
    }
    else {
      glDisable( GL_LIGHT1 ); 
      light1_spinner->disable();
    }
  }
  else if ( control == LIGHT0_INTENSITY_ID ) {
    float v[] = { 
      light0_diffuse[0],  light0_diffuse[1],
      light0_diffuse[2],  light0_diffuse[3] };
    
    v[0] *= light0_intensity;
    v[1] *= light0_intensity;
    v[2] *= light0_intensity;

    glLightfv(GL_LIGHT0, GL_DIFFUSE, v );
  }
  else if ( control == LIGHT1_INTENSITY_ID ) {
    float v[] = { 
      light1_diffuse[0],  light1_diffuse[1],
      light1_diffuse[2],  light1_diffuse[3] };
    
    v[0] *= light1_intensity;
    v[1] *= light1_intensity;
    v[2] *= light1_intensity;

    glLightfv(GL_LIGHT1, GL_DIFFUSE, v );
  }
  else if ( control == ENABLE_ID )
  {
    // glui2->enable();
  }
  else if ( control == DISABLE_ID )
  {
    // glui2->disable();
  }
  else if ( control == SHOW_ID )
  {
    // glui2->show();
  }
  else if ( control == HIDE_ID )
  {
    // glui2->hide();
  }
}

/**************************************** myGlutKeyboard() **********/

void myGlutKeyboard(unsigned char Key, int x, int y)
{
  switch(Key)
  {
  case 27: 
  case 'q':
    exit(0);
    break;
  };
  
  glutPostRedisplay();
}


/***************************************** myGlutMenu() ***********/
void myGlutMenu(int value)
{
  myGlutKeyboard(value, 0, 0);
}


/***************************************** myGlutIdle() ***********/

void myGlutIdle()
{
  /* According to the GLUT specification, the current window is 
     undefined during an idle callback.  So we need to explicitly change
     it if necessary */
  if (glutGetWindow() != main_window) {
    glutSetWindow(main_window);
  }

  /*  GLUI_Master.sync_live_all();  -- not needed - nothing to sync in this
                                       application  */

  glutPostRedisplay();
}

/***************************************** myGlutMouse() **********/

void myGlutMouse(int button, int button_state, int x, int y )
{
}


/***************************************** myGlutMotion() **********/

void myGlutMotion(int x, int y )
{
  glutPostRedisplay(); 
}

/**************************************** myGlutReshape() *************/

void myGlutReshape( int x, int y )
{
  int tx, ty, tw, th;
  GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
  glViewport( tx, ty, tw, th );

  xy_aspect = (float)tw / (float)th;

  glutPostRedisplay();
}


/************************************************** draw_axes() **********/
/* Disables lighting, then draws RGB axes                                */
void draw_axes(float scale)
{
  glDisable( GL_LIGHTING );

  glPushMatrix();
  glScalef( scale, scale, scale );

  glBegin( GL_LINES );
 
  glColor3f( 1.0, 0.0, 0.0 );
  glVertex3f( .8f, 0.05f, 0.0 );  glVertex3f( 1.0, 0.25f, 0.0 ); /* Letter X */
  glVertex3f( 0.8f, .25f, 0.0 );  glVertex3f( 1.0, 0.05f, 0.0 );
  glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 1.0, 0.0, 0.0 ); /* X axis      */

  glColor3f( 0.0, 1.0, 0.0 );
  glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 0.0, 1.0, 0.0 ); /* Y axis      */

  glColor3f( 0.0, 0.0, 1.0 );
  glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 0.0, 0.0, 1.0 ); /* Z axis    */
  glEnd();

  glPopMatrix();

  glEnable(GL_LIGHTING);
}

void draw_control_points(std::vector<ControlPointsMiner::ControlPoint*> points, const GLfloat* color)
{
	glDisable( GL_LIGHTING );

	glPushMatrix();
	glScalef( scale, scale, scale );

	glPointSize(10);
	glBegin(GL_POINTS);
	for (auto it=points.begin(); it != points.end(); it++) {
		if ((*it)->vertex) {
			glVertex3fv((*it)->vertex->pos.head<3>().data());
		}
	}
	glEnd();

	glPopMatrix();

	glEnable(GL_LIGHTING);
}

void draw_bounding_box(const Eigen::AlignedBox3f& boundingBox, const GLfloat* color)
{
	glDisable( GL_LIGHTING );

	glPushMatrix();
	glScalef( scale, scale, scale );

	glBegin( GL_LINES );

	Eigen::Vector3f bottomLeftFloor = boundingBox.corner(Eigen::AlignedBox3f::BottomLeftFloor);
	Eigen::Vector3f bottomRightFloor = boundingBox.corner(Eigen::AlignedBox3f::BottomRightFloor);
	Eigen::Vector3f topLeftFloor = boundingBox.corner(Eigen::AlignedBox3f::TopLeftFloor);
	Eigen::Vector3f topRightFloor = boundingBox.corner(Eigen::AlignedBox3f::TopRightFloor);
	Eigen::Vector3f bottomLeftCeil = boundingBox.corner(Eigen::AlignedBox3f::BottomLeftCeil);
	Eigen::Vector3f bottomRightCeil = boundingBox.corner(Eigen::AlignedBox3f::BottomRightCeil);
	Eigen::Vector3f topLeftCeil = boundingBox.corner(Eigen::AlignedBox3f::TopLeftCeil);
	Eigen::Vector3f topRightCeil = boundingBox.corner(Eigen::AlignedBox3f::TopRightCeil);

	glColor3fv(color);
	glVertex3fv(bottomLeftFloor.data());
	glVertex3fv(bottomRightFloor.data());

	glVertex3fv(bottomLeftFloor.data());
	glVertex3fv(topLeftFloor.data());

	glVertex3fv(bottomLeftFloor.data());
	glVertex3fv(bottomLeftCeil.data());

	glVertex3fv(bottomRightFloor.data());
	glVertex3fv(topRightFloor.data());

	glVertex3fv(bottomRightFloor.data());
	glVertex3fv(bottomRightCeil.data());

	glVertex3fv(topLeftFloor.data());
	glVertex3fv(topRightFloor.data());

	glVertex3fv(topLeftFloor.data());
	glVertex3fv(topLeftCeil.data());

	glVertex3fv(topRightFloor.data());
	glVertex3fv(topRightCeil.data());

	glVertex3fv(topRightCeil.data());
	glVertex3fv(topLeftCeil.data());

	glVertex3fv(topRightCeil.data());
	glVertex3fv(bottomRightCeil.data());

	glVertex3fv(bottomRightCeil.data());
	glVertex3fv(bottomLeftCeil.data());

	glVertex3fv(topLeftCeil.data());
	glVertex3fv(bottomLeftCeil.data());

	glEnd();

	glPopMatrix();

	glEnable(GL_LIGHTING);
}

/***************************************** myGlutDisplay() *****************/
void myGlutDisplay()
{
  glClearColor( .9f, .9f, .9f, 1.0f );
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-xy_aspect*.04, xy_aspect*.04, -.04, .04, .1, 15.0);

  glMatrixMode(GL_MODELVIEW);

  glLoadIdentity();
  glMultMatrixf(lights_rotation);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
  
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -2.6f);
  glTranslatef(obj_pos[0], obj_pos[1], -obj_pos[2]); 
  glMultMatrixf(view_rotate);

  glScalef(scale, scale, scale);

  if (data != NULL) {
	  GLfloat cpcolor[3] = {1.0, 1.0, 0.0};
  	  draw_control_points(controlPoints, cpcolor);

	  draw_axes(.52f);
	  if (curr_string == 2 || curr_string == 3) { // Wireframe
		  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	  } else {
		  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	  }

	  for(auto pit = data->parts.begin(); pit != data->parts.end(); pit++) {
		  GLfloat ncolor[3] = {0.0, 0.0, 1.0};
		  for (auto nit = (*pit)->neighbors.begin(); nit != (*pit)->neighbors.end(); nit++) {
			  draw_bounding_box((*nit)->boundingBox, ncolor);
		  }
		  GLfloat pcolor[3] = {0.0, 1.0, 0.0};
		  draw_bounding_box((*pit)->boundingBox, pcolor);
		  for(auto rit = (*pit)->regions.begin(); rit != (*pit)->regions.end(); rit++) {
			  GLfloat rcolor[3] = {1.0, 0.0, 0.0};
			  draw_bounding_box((*rit)->boundingBox, rcolor);
			  for(auto it = (*rit)->faces.begin(); it != (*rit)->faces.end(); it++) {
				  if (! (*it)->v1 || ! (*it)->v2 || ! (*it)->v3) {
					  continue;
				  }

				  glBegin(GL_TRIANGLES);
				  // Vertex 1
				  if (curr_string == 0 || curr_string == 3) { // Flat shaped
					  glNormal3f((*it)->normal.x(), (*it)->normal.y(), (*it)->normal.z());
				  }
				  else if (curr_string == 1) {
					  glNormal3f((*it)->v1->normal.x(), (*it)->v1->normal.y(), (*it)->v1->normal.z());
				  }
				  glVertex3f((*it)->v1->pos.x(),
						  (*it)->v1->pos.y(),
						  (*it)->v1->pos.z());

				  // Vertex 2
				  if (curr_string == 0 || curr_string == 3) { // Flat shaped
					  glNormal3f((*it)->normal.x(), (*it)->normal.y(), (*it)->normal.z());
				  }
				  else if (curr_string == 1) {
					  glNormal3f((*it)->v2->normal.x(), (*it)->v2->normal.y(), (*it)->v2->normal.z());
				  }
				  glVertex3f((*it)->v2->pos.x(),
						  (*it)->v2->pos.y(),
						  (*it)->v2->pos.z());

				  // Vertex 3
				  if (curr_string == 0 || curr_string == 3) { // Flat shaped
					  glNormal3f((*it)->normal.x(), (*it)->normal.y(), (*it)->normal.z());
				  }
				  else if (curr_string == 1) {
					  glNormal3f((*it)->v3->normal.x(), (*it)->v3->normal.y(), (*it)->v3->normal.z());
				  }
				  glVertex3f((*it)->v3->pos.x(),
						  (*it)->v3->pos.y(),
						  (*it)->v3->pos.z());

				  glEnd();
			  }
		  }
	  }

/*
	  for(auto pit = data2->parts.begin(); pit != data2->parts.end(); pit++) {
		  GLfloat pcolor[3] = {0.0, 1.0, 0.0};
//		  if ((*pit)->type == Data::Part::Type::SEAT_SHEET) {
			  draw_bounding_box((*pit)->boundingBox, pcolor);
			  for(auto rit = (*pit)->regions.begin(); rit != (*pit)->regions.end(); rit++) {
				  GLfloat rcolor[3] = {1.0, 0.0, 0.0};
				  draw_bounding_box((*rit)->boundingBox, rcolor);
				  for(auto it = (*rit)->faces.begin(); it != (*rit)->faces.end(); it++) {
					  if (! (*it)->v1 || ! (*it)->v2 || ! (*it)->v3) {
						  continue;
					  }

					  glBegin(GL_TRIANGLES);
					  // Vertex 1
					  if (curr_string == 0 || curr_string == 3) { // Flat shaped
						  glNormal3f((*it)->normal.x(), (*it)->normal.y(), (*it)->normal.z());
					  }
					  else if (curr_string == 1) {
						  glNormal3f((*it)->v1->normal.x(), (*it)->v1->normal.y(), (*it)->v1->normal.z());
					  }
					  glVertex3f((*it)->v1->pos.x(),
							  (*it)->v1->pos.y(),
							  (*it)->v1->pos.z());

					  // Vertex 2
					  if (curr_string == 0 || curr_string == 3) { // Flat shaped
						  glNormal3f((*it)->normal.x(), (*it)->normal.y(), (*it)->normal.z());
					  }
					  else if (curr_string == 1) {
						  glNormal3f((*it)->v2->normal.x(), (*it)->v2->normal.y(), (*it)->v2->normal.z());
					  }
					  glVertex3f((*it)->v2->pos.x(),
							  (*it)->v2->pos.y(),
							  (*it)->v2->pos.z());

					  // Vertex 3
					  if (curr_string == 0 || curr_string == 3) { // Flat shaped
						  glNormal3f((*it)->normal.x(), (*it)->normal.y(), (*it)->normal.z());
					  }
					  else if (curr_string == 1) {
						  glNormal3f((*it)->v3->normal.x(), (*it)->v3->normal.y(), (*it)->v3->normal.z());
					  }
					  glVertex3f((*it)->v3->pos.x(),
							  (*it)->v3->pos.y(),
							  (*it)->v3->pos.z());

					  glEnd();
				  }
///			  }
		  }
	  }
*/
  }

  glEnable( GL_LIGHTING );
  glutSwapBuffers(); 
}


/**************************************** main() ********************/

int main(int argc, char* argv[])
{
  parser = new SMFParser();
  data2 = parser->load("ChairA.obj");
//  data2 = parser->load("SimpleChair1.obj");
//  data = parser->load("SwivelChair04.obj");
  data = parser->load("chair0003.obj");
  data1 = data;

  // mixer = new SimpleBox();
  // mixer = new ClosestConnectingPoints();
  // mixer = new HullGridPoints();
  // mixer = new BoxIntersectionPoints();

  std::vector<ControlPointsMiner*> controlPointsMiners;
  controlPointsMiners.push_back(new HullGridPoints(10, 0, 10));
  controlPointsMiners.push_back(new BoxIntersectionPoints());

  mixer = new KNNWeightedInterpolation(controlPointsMiners, 8, 1.0);
  data = mixer->mix(data1, data2);
  controlPoints = mixer->totalControlPoints;

  // Data::Part* back1 = data1->findPartByType(Data::Part::Type::BACK_SHEET);
  // Data::Part* back2 = data2->findPartByType(Data::Part::Type::BACK_SHEET);
  // HullGridPoints* hullGridPoints = new HullGridPoints();
  // BoxIntersectionPoints* boxIntersectionPoints = new BoxIntersectionPoints();
  // ClosestConnectingPoints* closestConnectingPoints = new ClosestConnectingPoints();
  // EightPoints* eightPoints = new EightPoints();
  // controlPoints = eightPoints->findControlPoints(back1, back2);
  // std::cout << controlPoints.size() << std::endl;

//  data->findPartsNeighborsByBoxIntersection();
//  for (auto pit = data->parts.begin(); pit != data->parts.end(); pit++) {
//  	for (auto it = (*pit)->neighbors.begin(); it != (*pit)->neighbors.end(); it++) {
//		controlPoints.insert(controlPoints.end(), (*it)->vertices.begin(), (*it)->vertices.end());
//	}
//  }

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowPosition(50, 50);
  glutInitWindowSize(800, 800);
 
  main_window = glutCreateWindow("Assignment 1");
  glutDisplayFunc(myGlutDisplay);
  GLUI_Master.set_glutReshapeFunc(myGlutReshape);
  GLUI_Master.set_glutKeyboardFunc(myGlutKeyboard);
  GLUI_Master.set_glutSpecialFunc(NULL);
  GLUI_Master.set_glutMouseFunc(myGlutMouse);
  glutMotionFunc(myGlutMotion);

  glEnable(GL_LIGHTING);
  glEnable(GL_NORMALIZE);

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

  glEnable(GL_LIGHT1);
  glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

  glEnable(GL_DEPTH_TEST);

  printf("GLUI version: %3.2f\n", GLUI_Master.get_version());

  glui = GLUI_Master.create_glui_subwindow(main_window, 
					   GLUI_SUBWINDOW_RIGHT);

  file_browser = new GLUI_FileBrowser(glui, "Open a File", GLUI_PANEL_RAISED, OPEN_ID, control_callback);
  file_browser->set_allow_change_dir(true);
  file_browser->set_h(100);

  new GLUI_StaticText(glui, "");

  new GLUI_Button(glui, "Save", 0, control_callback);

  new GLUI_StaticText(glui, "");
 
  display_list = new GLUI_Listbox(glui, "Display:", &curr_string);
  for(int i=0; i<4; i++) {
    display_list->add_item(i, display_options[i]);
  }

  new GLUI_StaticText(glui, "");
  GLUI_Rotation *view_rot1 = new GLUI_Rotation(glui, "Objects", view_rotate);
  view_rot1->set_spin(1.0);

  new GLUI_StaticText(glui, "");
  GLUI_Translation *trans_xy = new GLUI_Translation(glui, "Objects XY", GLUI_TRANSLATION_XY, obj_pos);
  trans_xy->set_speed(.005);

  new GLUI_StaticText(glui, "");
  GLUI_Translation *trans_x = new GLUI_Translation(glui, "Objects X", GLUI_TRANSLATION_X, obj_pos);
  trans_x->set_speed(.005);

  new GLUI_StaticText(glui, "");
  GLUI_Translation *trans_y = new GLUI_Translation( glui, "Objects Y", GLUI_TRANSLATION_Y, &obj_pos[1]);
  trans_y->set_speed(.005);

  new GLUI_StaticText(glui, "");
  GLUI_Translation *trans_z = new GLUI_Translation( glui, "Objects Z", GLUI_TRANSLATION_Z, &obj_pos[2] );
  trans_z->set_speed(.005);

  new GLUI_StaticText(glui, "");
  
  glui->set_main_gfx_window(main_window);

  /*** Disable/Enable buttons ***/
  //new GLUI_Button( glui, "Disable movement", DISABLE_ID, control_callback );
  //new GLUI_Button( glui, "Enable movement", ENABLE_ID, control_callback );
  //new GLUI_Button( glui, "Hide", HIDE_ID, control_callback );
  //new GLUI_Button( glui, "Show", SHOW_ID, control_callback );

  // new GLUI_StaticText( glui, "" );

  /****** A 'quit' button *****/
  new GLUI_Button(glui, "Quit", 0, (GLUI_Update_CB)exit);

  /**** Link windows to GLUI, and register idle callback ******/
  glui->set_main_gfx_window(main_window);

  /*** Create the bottom subwindow ***/
//   glui2 = GLUI_Master.create_glui_subwindow(main_window, 
//                                            GLUI_SUBWINDOW_BOTTOM);
//  glui2->set_main_gfx_window(main_window);

#if 0
  /**** We register the idle callback with GLUI, *not* with GLUT ****/
  GLUI_Master.set_glutIdleFunc( myGlutIdle );
#endif

  /**** Regular GLUT main loop ****/
  
  glutMainLoop();

  delete parser;
  if (data != NULL) {
    delete data;
  }

  return EXIT_SUCCESS;
}
