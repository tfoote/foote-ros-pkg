#! /usr/bin/env python

"""
usage: %(progname)s file.lxf
"""

import os, sys, getopt
import zipfile
import xml.dom.minidom
import numpy
from numpy.linalg import inv

import transformations as TF

def parseInts(s):
  if s:
    return [int(x) for x in s.split(',')]
  else:
    return None

def parseFloats(s):
  if s:
    return [float(x) for x in s.split(',')]
  else:
    return None

link_template = """
    <link name="ref_%(refID)s_link">
      <inertial>
        <mass value="0.010000" />
        <!-- center of mass (com) is defined w.r.t. link local coordinate system -->
        <origin xyz="0.0 0.0 0" />
        <inertia  ixx="0.01" ixy="0.0"  ixz="0.0"  iyy="0.01"  iyz="0.0"  izz="0.01" />
      </inertial>
      <visual>
        <!-- visual origin is defined w.r.t. link local coordinate system -->
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <mesh filename="%(mesh)s" scale="0.0004 0.0004 0.0004"/>
        </geometry>
      </visual>
      <collision>
        <!-- collision origin is defined w.r.t. link local coordinate system -->
        <origin xyz="%(bound_x)s %(bound_y)s %(bound_z)s" rpy="%(bound_roll)s %(bound_pitch)s %(bound_yaw)s" />
        <geometry>
          <box size="%(dim_x)s %(dim_y)s %(dim_z)s"/>
        </geometry>
      </collision>
    </link>
"""

joint_template = """
    <joint name="ref_%(refID)s_joint" type="%(joint_type)s">
      <parent link="%(parent_link)s"/>
      <child link="%(child_link)s"/>
      <origin xyz="%(origin_x)s %(origin_y)s %(origin_z)s" rpy="%(origin_roll)s %(origin_pitch)s %(origin_yaw)s" />
      <axis xyz="%(axis_x)s %(axis_y)s %(axis_z)s" />
    </joint>
"""

def parseLXFML(handle, name):
  #print "Parsing file %s" % handle.name
  print "<robot name=%s>" % name
  ldraw = open('ldraw.xml','r') #we need to add the ldraw.xml transformations
  ldraw_doc = xml.dom.minidom.parse(ldraw)
  lxf_doc = xml.dom.minidom.parse(handle)

  bricks = {}
  parts = {}
  bones = {}
  for brick in lxf_doc.getElementsByTagName('Brick'):
    b = {}
    b['parts'] = []
    b['designID'] = int(brick.getAttribute('designID'))
    i = parseInts(brick.getAttribute('itemNos'))
    if i:
      b['itemNos'] = i
    bricks[int(brick.getAttribute('refID'))] = b

    for part in brick.getElementsByTagName('Part'):
      p = {}
      p['bones'] = []
      p['designID'] = part.getAttribute('designID')
      p['materials'] = parseInts(part.getAttribute('materials'))
      parts[int(part.getAttribute('refID'))] = p
      b['parts'].append(p)
      p['parent'] = b

      for bone in part.getElementsByTagName('Bone'):
        d = {}
        d['transformation'] = parseFloats(bone.getAttribute('transformation'))
        bones[int(bone.getAttribute('refID'))] = d
        p['bones'].append(d)
        d['parent'] = p

  rigids = {}
  for rigid in lxf_doc.getElementsByTagName('Rigid'):
    r = {}
    r['refID'] = int(rigid.getAttribute('refID'))
    r['transformation'] = parseFloats(rigid.getAttribute('transformation'))
    r['boneRefs'] = parseInts(rigid.getAttribute('boneRefs'))
    rigids[r['refID']] = r
  
  joints = []
  for joint in lxf_doc.getElementsByTagName('Joint'):
    j = []
    for rigid_ref in joint.getElementsByTagName('RigidRef'):
      r = {}
      r['rigidRef'] = int(rigid_ref.getAttribute('rigidRef'))
      r['a'] = parseFloats(rigid_ref.getAttribute('a'))
      r['z'] = parseFloats(rigid_ref.getAttribute('z'))
      r['t'] = parseFloats(rigid_ref.getAttribute('t'))
      j.append(r)
    joints.append(j)

  transformations = {}
  for transformation in ldraw_doc.getElementsByTagName('Transformation'):
    t = {}
    t['ldraw'] = transformation.getAttribute('ldraw').strip('.dat')
    t['t'] = [float(transformation.getAttribute('tx')), float(transformation.getAttribute('ty')), float(transformation.getAttribute('tz'))]
    t['axis'] = [float(transformation.getAttribute('ax')), float(transformation.getAttribute('ay')), float(transformation.getAttribute('az'))]
    t['angle'] = float(transformation.getAttribute('angle'))
    transformations[t['ldraw']] = t

  for refID in sorted(rigids.keys()):
    rigid = rigids[refID]
    designID = bones[rigid['boneRefs'][0]]['parent']['parent']['designID']
    d = {
      'refID' : refID,
      'mesh' : "package://nxt_description/meshes/parts/%s.dae" % designID,
      'bound_x' : 0,
      'bound_y' : 0,
      'bound_z' : 0,
      'bound_roll' : 0,
      'bound_pitch' : 0,
      'bound_yaw' : 0,
      'dim_x' : 0,
      'dim_y' : 0,
      'dim_z' : 0,
    }
    print link_template % d

  for refID, joint_list in enumerate(joints):
    joint_type = "fixed"
    child_refID = joint_list[0]['rigidRef']
    parent_refID = joint_list[1]['rigidRef']
    #all units are in CM
    #the models are in LDU which are 0.4mm 
    #let's compute some transforms
    c_to_ldraw = homogeneous_matrix(TF.rotation_matrix(transformations[str(bricks[child_refID]['designID'])]['angle'],
                                    numpy.array(transformations[str(bricks[child_refID]['designID'])]['axis'])),
                                    numpy.transpose(numpy.array(transformations[str(bricks[child_refID]['designID'])]['t'])))
    #print c_to_ldraw
    p_to_ldraw = homogeneous_matrix(TF.rotation_matrix(transformations[str(bricks[parent_refID]['designID'])]['angle'],
                                    numpy.array(transformations[str(bricks[parent_refID]['designID'])]['axis'])),
                                    numpy.zeros((1,3))) 
                                    #numpy.transpose(numpy.array(transformations[str(bricks[parent_refID]['designID'])]['t'])))
    world_to_c = homogeneous_matrix(numpy.array(rigids[child_refID]['transformation'][:9]).reshape(3,3), rigids[child_refID]['transformation'][9:12]) #this should be correct looking at models
    world_to_p = homogeneous_matrix(numpy.array(rigids[parent_refID]['transformation'][:9]).reshape(3,3), rigids[parent_refID]['transformation'][9:12]) #this should be correct looking at models
    c_world_to_ldraw = numpy.dot(world_to_c, c_to_ldraw)
    p_world_to_ldraw = numpy.dot(world_to_p, p_to_ldraw)
    #now let's get the stuff for the URDF
    p_to_c = numpy.dot(inv(p_world_to_ldraw), c_world_to_ldraw)
    rpy = TF.euler_from_matrix(p_to_c) 

    d = {
      'refID' : refID,
      'joint_type' : joint_type,
      'parent_link' : 'ref_%s_link' % parent_refID,
      'child_link' : 'ref_%s_link' % child_refID,
      'origin_x' : '%s' % str(float(p_to_c[0,3])/100.0), 
      'origin_y' : '%s'% str(float(p_to_c[1,3])/100.0),
      'origin_z' : '%s' % str(float(p_to_c[2,3])/100.0),
      'origin_roll' : '%s' % rpy[0],
      'origin_pitch' : '%s' % rpy[1],
      'origin_yaw' : '%s' % rpy[2],
      'axis_x' : 0, 'axis_y' : 0, 'axis_z' : 0,
    }
    print joint_template % d
  print "</robot>"

def homogeneous_matrix(rot, trans):
  if rot.shape != (3,3):
    rot[:3,3] = trans
    return rot
  else:  
    tmp = numpy.ones((4,4))
    tmp[:3,:3] = rot
    tmp[:3,3] = trans
    tmp[3,:3] = numpy.zeros((1,3))
    return tmp

def handleLXF(filename):
  z = zipfile.ZipFile(filename)
  for item in z.namelist():
    if item.endswith('.LXFML'):
      f = z.open(item)
      parseLXFML(f,filename)

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "debug"])

  if len(args) == 0:
    usage(progname)
    return
  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()

  for filename in args:
    try:
      handleLXF(filename)
    except Exception, reason:
      print >> sys.stderr, "Unable to handle '%s': %s" % (filename, reason)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
