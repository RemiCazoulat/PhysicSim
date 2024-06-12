using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

public class Parser
{

    public Parser()
    {
        //float valor = float.Parse("1.425", locale);
    }

    public int[] ParseInt (TextAsset fileName, bool debugMode) {
        var locale = new CultureInfo("en-US");

        // Ayuda TextAsset: https://docs.unity3d.com/ScriptReference/TextAsset.html
        // Ayuda Unity de String https://docs.unity3d.com/ScriptReference/String.html
        // Ayuda MSDN de String https://docs.microsoft.com/en-us/dotnet/api/system.string?redirectedfrom=MSDN&view=netframework-4.8
        // Ayuda MSDN de String.Split https://docs.microsoft.com/en-us/dotnet/api/system.string.split?view=netframework-4.8

        string[] textString = fileName.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);
        //int numNodes = int.Parse(textString[0]);

        // NOTA: 
        // Para parsear números flotantes hay que tener en
        // cuenta el formato de número en el que está escrito.
        // Para ello hay que instanciar un objeto de la clase
        // CultureInfo que almacena información de localización. 
        // Los números con "." como separador decimal como 1.425
        // tienen localización de EEUU, "en-US".
        /*
        CultureInfo locale = new CultureInfo("en-US");
        float valor = float.Parse("1.425", locale);
        */

        var resultList = new List<int>();
        
        for (int i = 0; i < textString.Length; i++)
        {
            if (int.TryParse(textString[i], NumberStyles.Float, locale, out int isGood))
            {
                resultList.Add(isGood);
            }
        }
        var result = resultList.ToArray();

        if (!debugMode) return result;
        
        string debug = "Parse int debug : \n";
        for (int i = 0; i < textString.Length; i++)
        {
            debug += "line " + i + " : " + textString[i] + "\n";

        }
        Debug.Log(debug);
        return result;
    }
    public float[] ParseFloat (TextAsset fileName, bool debugMode) {
        IFormatProvider locale = new CultureInfo("en-US");


        string[] textString = fileName.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);

        //var result = new float[textString.Length];
        var resultList = new List<float>();
        
        for (int i = 0; i < textString.Length; i++)
        {
            if (float.TryParse(textString[i], NumberStyles.Float, locale, out float isGood))
            {
                resultList.Add(isGood);
            }
        }

        var result = resultList.ToArray();
        
        if (!debugMode) return result;
        
        string debug = "Parse float debug : \n";
        for (int i = 0; i < result.Length; i++)
        {
            debug += "line " + i + " : " + result[i] + "\n";

        }
        Debug.Log(debug);
        return result;
    }

}