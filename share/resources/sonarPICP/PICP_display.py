from mayavi import mlab
from abc import ABC, abstractmethod

class PICP_display_base():
    """
        GUI for displaying the results of pIC algorithms at each step.
        Base pIC display class 
    """

    def __init__(self, showAssociations):
        """
            Constructor

            Parameters
            ----------
            showAssociations : bool
                               flag to show/hide line between associated points
        """

        # If true, draw lines between associated points 
        self.showAssociations = showAssociations

        # Color for associated points in the target point cloud 
        # (same notation as papers in the doc folder)
        self.a_pts_color = (1, 0, 0)

        # Color for associated points in the reference point cloud 
        # (same notation as papers in the doc folder)
        self.c_pts_color = (0, 0, 1)

        # Color for the transformed reference point cloud with the current estimated transformation 
        self.n_pts_color = (0, 1, 0)

        # Color for the transformed reference point cloud with the groud-truth transformation (if available)
        self.n_pts_GT_color = (1, 0, 1)

        # Color for the transformed reference point cloud with the initial estimated transformation
        self.n_pts_init_color = (0, 1, 1)

        # Color of the line when drawing lines between associated points
        self.assoc_line_color= (0, 0, 0)

        # Color of points projected in tangent planes of the target point cloud
        # (pointToplane association only)
        self.pointToPlaneProjections_color = (1, 0.64, 0)
        
        # Color of the normal (pointToPlane association only)
        self.normal_color = (0.5, 0.5, 0.5)

        # Size of the points
        self.scale_factor = 0.1

        # Type of association (pointToPoint or pointToPlane)
        self.associationType = "pointToPoint" # "pointToPlane"

        # Figure
        self.fig = mlab.figure(bgcolor=(1,1,1))

    @abstractmethod
    def drawLinesBetweenPoints(self, points1, points2, color):
        pass

    @abstractmethod
    def drawPointCloud(self, pointCloud, color):
        pass

class PICP_display_2D(PICP_display_base):
    """ 
        pIC display class in the 2D case 
    """

    def __init__(self, showAssociations):
        """
            Constructor

            Parameters
            ----------
            showAssociations : bool
                               flag to show/hide line between associated points
        """
        PICP_display_base.__init__(self, showAssociations)

    def drawLinesBetweenPoints(self, points1, points2, color):
        """ 
            Draw lines between corresponding points in points1 and points2 

            Parameters
            ---------
            points1 : (n,3) array
                      array of points
            points2 : (n,3) array
                      array of points
            color : (3) tuple
                    line RGB color (values in [0,1])
        """

        for pt_A, pt_N in zip(points1, points2):
            mlab.plot3d([pt_A[0], pt_N[0]], [pt_A[1], pt_N[1]], [0, 0], color=color)
    
    def drawPointCloud(self, pointCloud, color):
        """ 
            Draw a point cloud

            Parameters
            ----------
            pointCloud : (n,3) array
                         array of points
            color : (3) tuple
                    point RGB color (values in [0,1])
        """

        # Just draw it in 3D in the plane z=0
        null_z = np.zeros((pointCloud.shape[0],))
        mlab.points3d(pointCloud[:, 0], pointCloud[:, 1], null_z,
                      color=color,
                      scale_factor=self.scale_factor)

class PICP_display():
    """ 
        pIC display class in the 3D case
    """

    def __init__(self, showAssociations):
        """
            Constructor

            Parameters
            ----------
            showAssociations : bool
                               flag to show/hide line between associated points
        """
        PICP_display_base.__init__(self, showAssociations)

    def drawLinesBetweenPoints(self, points1, points2, color):
        """ 
            Draw lines between corresponding points in points1 and points2 

            Parameters
            ----------
            points1 : (n,3) array
                      array of points
            points2 : (n,3) array
                      array of points
            color : (3) tuple
                    line RGB color (values in [0,1])
        """

        for pt_A, pt_N in zip(points1, points2):
            mlab.plot3d([pt_A[0], pt_N[0]], [pt_A[1], pt_N[1]], [pt_A[2], pt_N[2]], color=color)

    def drawPointCloud(self, pointCloud, color):
        """ 
            Draw a point cloud 

            Parameters
            ----------
            pointCloud : (n,3) array
                         array of points
            color : (3) tuple
                    points RGB color (values in [0,1])
        """

        mlab.points3d(pointCloud[:, 0], pointCloud[:, 1], pointCloud[:, 2],
                      color=color,
                      scale_factor=self.scale_factor)

    def drawNormals(self, points, normals, color):
        """ 
            Draw normals at points (only used with pointToPlane association) 

            Parameters
            ----------
            points : (n,3) array
                     array of points
            normals : (n,3) array
                      array of normals at each point
            color : (3) tuple
                    normals RGB color (values in [0,1])
        """

        # Length of the normals
        length = 4.
        mlab.quiver3d(points[:,0], points[:,1], points[:,2],
                      length*normals[:,0], length*normals[:,1], length*normals[:,2],
                      color=color,
                      scale_factor=self.scale_factor)

    def display(self, a_pts_array, c_pts_array, n_pts_array,
                n_pts_GT=None, n_pts_init=None, a_pts_array_assoc = None, assoc_index_N = None,
                normals = None):
        """ 
            Main function of the class. Display the different point clouds 

            Parameters
            ----------
            a_pts_array : dict with at least ["mean"] ((n,3) array) 
                          first point cloud 
            c_pts_array : dict with at least ["mean"] ((m,3) array)
                          second point cloud
            n_pts_array :  dict with at least ["mean"] ((m,3) array)
                          second point cloud after application of the transformation ie n_i = q + c_i
            n_pts_GT : (m,3) array
                       second point cloud after application of the ground truth transformation ie n_i = q_gt + c_i
            n_pts_init :  dict with at least ["mean"] ((m,3) array)
                         second point cloud after application of the initial transformation ie n_i = q0 + c_i
            a_pts_array_assoc : dict with at least ["mean"] ((l,3) array)
                                subset of first point cloud currently associated
            assoc_index_N : (l) array of int
                            indexes of second point cloud associated
            normals : (n,3) array
                      normals at the first point cloud
        """

        # Reset the display
        mlab.clf()

        # Draw the different point clouds
        self.drawPointCloud(a_pts_array["mean"], self.a_pts_color)
        self.drawPointCloud(c_pts_array["mean"], self.c_pts_color)
        self.drawPointCloud(n_pts_array["mean"], self.n_pts_color)

        # Draw the idx of each point for the target point cloud
        # for a_pt, arcIdx in zip(a_pts_array['mean'], a_pts_array['arcIdx']):
        #    mlab.text3d(a_pt[0], a_pt[1], a_pt[2], str(arcIdx), figure=fig, color=(0,0,0),scale=scale_factor)

        # If provided, draw the reference cloud transformed with the ground truth transformation
        if(n_pts_GT is not None):
            self.drawPointCloud(n_pts_GT, self.n_pts_GT_color)

        # If provided, draw the reference point cloud transformed with the initial transformation
        if(n_pts_init is not None):
            self.drawPointCloud(n_pts_init["mean"], self.n_pts_init_color)

        # If required, draw lines between associated points 
        if (self.showAssociations and a_pts_array_assoc is not None and assoc_index_N is not None):
            self.drawLinesBetweenPoints(a_pts_array_assoc["mean"], n_pts_array["mean"][assoc_index_N], self.assoc_line_color)

        # Draw the projected point used for the optimization step
        if (self.associationType == "pointToPlane" and a_pts_array_assoc is not None):
            self.drawPointCloud(a_pts_array_assoc["mean"], self.pointToPlaneProjections_color)
        
        # Draw the normals to target point cloud if available
        if (normals is not None):
            self.drawNormals(a_pts_array_assoc["mean"], normals, self.normal_color)

        mlab.gcf().scene.parallel_projection = True
        mlab.orientation_axes()
        mlab.show(stop=True)
