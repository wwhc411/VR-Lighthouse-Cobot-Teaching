using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PluginVersion : MonoBehaviour
{
    private GameObject versionObject = null;
    private Text versionText = null;

    // Start is called before the first frame update
    [System.Obsolete]
    void Start()
    {
        try
        {
            versionObject = GameObject.Find("VersionText");
            versionText = versionObject.GetComponent<Text>();
        }
        finally
        {

        }
    }

    // Update is called once per frame
    void Update()
    {
        if (versionText != null)
        {
            versionText.text = "Plugin Version:1.3.5681"; 
        }
    }
}
