#include "step_to_stl.h"

// OCE library headers
#include <Interface_Static.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <TCollection_AsciiString.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopoDS_Shape.hxx>

#include <ros/console.h>

bool urdf_editor::codecs::convertStepToStl(const std::string& step_file_path, 
                                           const std::string& stl_file_path)
{
  STEPControl_Reader step_reader;
  Interface_Static::SetCVal("xstep.cascade.unit", "M"); // units in Meters

  const char* oce_units = Interface_Static::CVal("xstep.cascade.unit"); 
  ROS_INFO("OCE Interface Units: %s", oce_units);
  

  IFSelect_ReturnStatus ret = step_reader.ReadFile(step_file_path.c_str());

  step_reader.PrintCheckLoad(0, static_cast<IFSelect_PrintCount>(0));

  if (ret != IFSelect_RetDone) 
  {
    ROS_WARN("Unable to load STEP file '%s'", step_file_path.c_str());
    return false;
  }

  TColStd_SequenceOfAsciiString lengths, angles, solid_angles;
  step_reader.FileUnits(lengths, angles, solid_angles);

  ROS_INFO("STEP File Length Units:");
  for (int i = 0; i < lengths.Length(); ++i)
  {
    ROS_INFO("%d: %s", i, lengths(i+1).ToCString()); 
  }

  ROS_INFO("STEP File Angle Units:");
  for (int i = 0; i < angles.Length(); ++i)
  {
    ROS_INFO("%d: %s", i, angles(i+1).ToCString()); 
  }

  ROS_INFO("STEP File Solid-Angle Units:");
  for (int i = 0; i < solid_angles.Length(); ++i)
  {
    ROS_INFO("%d: %s", i, solid_angles(i+1).ToCString()); 
  }

  Standard_Integer n = step_reader.TransferRoots();
  TopoDS_Shape shape = step_reader.OneShape();

  // Our edition of OCE doesn't have a return code for the above write
  StlAPI_Writer writer;
  writer.ASCIIMode() = 0;
  writer.Write(shape, stl_file_path.c_str());
  return true;
}
