# Assuming you have not changed the general structure of the template no modification is needed in this file.
from . import commands
from .lib import fusionAddInUtils as futil
import adsk.core, adsk.fusion, traceback, math
import os
import tempfile
from xml.dom import minidom
import xml.etree.ElementTree as ET
import math
try:
    import numpy as np
except:
    # pipからnumpyをインストールする
    import subprocess
    import sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy"])
    import numpy as np
try:
    from scipy.spatial.transform import Rotation as R
except:
    # pipからscipyをインストールする
    import subprocess
    import sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy"])
    from scipy.spatial.transform import Rotation as R

UNIT_OF_LENGTH_RATIO = 1/100.0

def get_user_download_folder():
    """
        ユーザーのダウンロードフォルダを取得するための関数
    Returns:
        str: フォルダパス
    """
    # ユーザーフォルダのパスを取得
    user_folder = os.path.expanduser("~")
    folder = os.path.join(user_folder, "Downloads")
    return folder


def run(context):
    try:
        # This will run the start function in each of your commands as defined in commands/__init__.py
        commands.start()
        app = adsk.core.Application.get()
        ui = app.userInterface

        # Get the root component of the active design.
        design = app.activeProduct
        rootComp = design.rootComponent

        # コンポーネントツリーを再帰的に探索する
        component_tree = {"root": {"obj":rootComp, "parent":None, "trans":None, "children": []}}
        def traverse_component(component, parent, p=None, p_trans=None, p_rot=None):
            for child in component.occurrences:
                path = parent + "/" + child.name
                assert path not in component_tree, "Duplicate path: " + path
                try:
                    c_matrix = np.array(child.transform.asArray()).reshape(4,4)
                    c_trans = c_matrix[:3,3]
                    if p_rot is not None:
                        c_trans = p_rot.apply(c_trans)
                    c_rot = np.array(child.transform.asArray()).reshape(4,4)[:3,:3]
                    c_rot = R.from_matrix(c_rot)
                    #print(c_rot.as_euler('xyz'))
                    if "AngleA" in path:
                        pass
                except:
                    c_trans = None
                    c_rot = None
                if p_trans is None:
                    c_trans = c_trans
                    c_rot = c_rot
                elif c_trans is None:
                    c_trans = p_trans
                    c_rot = p_rot
                else:
                    c_trans = np.array([v1+v2 for v1,v2 in zip(p_trans, c_trans)])
                    c_rot = c_rot * p_rot
                component_tree[path] = {"obj":child, "trans":c_trans, "rot":c_rot, "parent":parent, "children": []}
                component_tree[parent]["children"].append(path)
                traverse_component(child.component, path, child, c_trans, c_rot)
        traverse_component(rootComp, 'root')
        #print(component_tree)

        # ツリーの中からボディのみを抽出する
        body_tree = {}
        def traverse_body(path):
            component = component_tree[path]["obj"]
            for body in component.bRepBodies:
                body_tree[path] = body
                break
            for child in component_tree[path]["children"]:
                traverse_body(child)
        traverse_body('root')

        # revisionIdを使って同一ボディーコンポーネントをまとめる
        revisionId_tree = {}
        for path, body in body_tree.items():
            revisionId = body.revisionId
            if revisionId not in revisionId_tree:
                revisionId_tree[revisionId] = {"stl":None, "paths":[]}
            revisionId_tree[revisionId]["paths"].append(path)
        #for revisionId, paths in revisionId_tree.items():
        #    print(revisionId, paths)

        # 各ボディーコンポーネントをstlファイルとして書き出す
        dst_dir = os.path.join(get_user_download_folder(), component_tree["root"]["obj"].name, 'meshes')
        os.makedirs(dst_dir.encode("utf-8"), exist_ok=True)
        for revisionId in revisionId_tree.keys():
            for path in revisionId_tree[revisionId]["paths"]:
                try:
                    body = body_tree[path]
                    stl_name = "__".join([_p.replace(":","_") for _p in path.split("/")[1:]])
                    stl_filename = os.path.join(dst_dir, stl_name + ".stl")
                    print("Exporting", stl_filename)
                    exportMgr = adsk.fusion.ExportManager.cast(design.exportManager)
                    stlOptions = exportMgr.createSTLExportOptions(body, stl_filename)            
                    stlOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    stlOptions.isOneFilePerBody = True
                    stlOptions.sendToPrintUtility = False
                    exportMgr.execute(stlOptions)
                    adsk.doEvents()
                    revisionId_tree[revisionId]["stl"] = stl_filename
                except Exception as e:
                    print("Failed to export", path, e)
                break
        #print(revisionId_tree)

        # URDFファイルを書き出す
        dst_dir = os.path.dirname(dst_dir)
        urdf_filename = os.path.join(dst_dir, "model.urdf")
        robot = ET.Element('robot', name=component_tree["root"]["obj"].name)
        for path, body in body_tree.items():
            link_name = "__".join([_p.replace(":","_") for _p in path.split("/")[1:]])
            link = ET.SubElement(robot, 'link', name=link_name)
            visual = ET.SubElement(link, 'visual', name=link_name + "_visual")
            collision = ET.SubElement(link, 'collision', name=link_name + "_collision")
            trans = component_tree[path]['trans']
            rot = component_tree[path]['rot']
            if trans is None:
                xyz = "0 0 0"
            else:
                xyz = " ".join(["{:.4f}".format(v * UNIT_OF_LENGTH_RATIO) for v in trans])
            if rot is None:
                rpy = "0 0 0"
            else:
                rpy = rot.as_euler('xyz')
                #rpy[-1] += 0.5 * math.pi
                rpy = " ".join(["{:.4f}".format(v) for v in rpy])
            #occurrences = rootComp.occurrencesByComponent(body.parentComponent)
            origin = ET.SubElement(visual, 'origin', xyz=xyz, rpy=rpy)
            geometry = ET.SubElement(visual, 'geometry')
            mesh = ET.SubElement(geometry, 'mesh', filename=revisionId_tree[body.revisionId]["stl"].replace(dst_dir, "").replace("\\","/").replace("/meshes","meshes"), scale="0.001 0.001 0.001")
            origin = ET.SubElement(collision, 'origin', xyz=xyz, rpy=rpy)
            geometry = ET.SubElement(collision, 'geometry')
            mesh = ET.SubElement(geometry, 'mesh', filename=revisionId_tree[body.revisionId]["stl"].replace(dst_dir, "").replace("\\","/").replace("/meshes","meshes"), scale="0.001 0.001 0.001")
            material = ET.SubElement(visual, 'material', name=body.material.id)
            color = [float(v)/255 for v in body.appearance.name.replace("Opaque(", "").replace(")", "").split(",")]
            color = ET.SubElement(material, 'color', rgba=" ".join(["{:.4f}".format(v) for v in color]+["1.0"]))
        doc = minidom.parseString(ET.tostring(robot, 'utf-8'))
        with open(urdf_filename,'w') as f:
            doc.writexml(f, encoding='utf-8', newl='\n', indent='', addindent='  ')
            
        ui.messageBox("Exported to " + urdf_filename)

    except:
        print(traceback.format_exc())
        futil.handle_error('run')
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    try:
        # Remove all of the event handlers your app has created
        futil.clear_handlers()

        # This will run the start function in each of your commands as defined in commands/__init__.py
        commands.stop()

    except:
        futil.handle_error('stop')