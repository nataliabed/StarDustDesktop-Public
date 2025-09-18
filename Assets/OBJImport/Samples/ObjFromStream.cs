using Dummiesman;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.Networking;
using System.Collections;

public class ObjFromStream : MonoBehaviour {
	void Start () {
        StartCoroutine(LoadObjFromURL());
	}

    private IEnumerator LoadObjFromURL()
    {
        using (UnityWebRequest request = UnityWebRequest.Get("https://people.sc.fsu.edu/~jburkardt/data/obj/lamp.obj"))
        {
            yield return request.SendWebRequest();
            
            if (request.result == UnityWebRequest.Result.Success)
            {
                //create stream and load
                var textStream = new MemoryStream(Encoding.UTF8.GetBytes(request.downloadHandler.text));
                var loadedObj = new OBJLoader().Load(textStream);
            }
            else
            {
                Debug.LogError("Failed to load OBJ: " + request.error);
            }
        }
    }
}
