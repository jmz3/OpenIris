﻿//<auto-generated />
namespace OpenIris.UI
{
    partial class EyeTrackingPipelinePupilCRQuickSettings
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }

            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.trackBarPupilThreshold = new System.Windows.Forms.TrackBar();
            this.label2 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.trackBarReflectionThreshold = new System.Windows.Forms.TrackBar();
            this.textBoxPupilThreshold = new System.Windows.Forms.TextBox();
            this.textBoxReflectionThreshold = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarPupilThreshold)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarReflectionThreshold)).BeginInit();
            this.SuspendLayout();
            // 
            // trackBarPupilThreshold
            // 
            this.trackBarPupilThreshold.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.trackBarPupilThreshold.Location = new System.Drawing.Point(162, 152);
            this.trackBarPupilThreshold.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.trackBarPupilThreshold.Maximum = 255;
            this.trackBarPupilThreshold.Name = "trackBarPupilThreshold";
            this.trackBarPupilThreshold.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.trackBarPupilThreshold.Size = new System.Drawing.Size(345, 69);
            this.trackBarPupilThreshold.TabIndex = 23;
            this.trackBarPupilThreshold.TickFrequency = 10;
            this.trackBarPupilThreshold.TickStyle = System.Windows.Forms.TickStyle.None;
            this.trackBarPupilThreshold.Value = 60;
            this.trackBarPupilThreshold.Scroll += new System.EventHandler(this.TrackBarPupilThreshold_Scroll);
            // 
            // label2
            // 
            this.label2.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(8, 152);
            this.label2.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(117, 20);
            this.label2.TabIndex = 24;
            this.label2.Text = "Pupil Threshold";
            // 
            // label5
            // 
            this.label5.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(8, 208);
            this.label5.Margin = new System.Windows.Forms.Padding(4, 0, 4, 0);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(155, 20);
            this.label5.TabIndex = 24;
            this.label5.Text = "Reflection Threshold";
            // 
            // trackBarReflectionThreshold
            // 
            this.trackBarReflectionThreshold.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.trackBarReflectionThreshold.Location = new System.Drawing.Point(162, 208);
            this.trackBarReflectionThreshold.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.trackBarReflectionThreshold.Maximum = 255;
            this.trackBarReflectionThreshold.Name = "trackBarReflectionThreshold";
            this.trackBarReflectionThreshold.RightToLeft = System.Windows.Forms.RightToLeft.No;
            this.trackBarReflectionThreshold.Size = new System.Drawing.Size(345, 69);
            this.trackBarReflectionThreshold.TabIndex = 23;
            this.trackBarReflectionThreshold.TickFrequency = 10;
            this.trackBarReflectionThreshold.TickStyle = System.Windows.Forms.TickStyle.None;
            this.trackBarReflectionThreshold.Value = 60;
            this.trackBarReflectionThreshold.Scroll += new System.EventHandler(this.TrackBarReflectionThreshold_Scroll);
            // 
            // textBoxPupilThreshold
            // 
            this.textBoxPupilThreshold.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.textBoxPupilThreshold.BackColor = System.Drawing.SystemColors.Control;
            this.textBoxPupilThreshold.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBoxPupilThreshold.Location = new System.Drawing.Point(513, 152);
            this.textBoxPupilThreshold.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxPupilThreshold.Name = "textBoxPupilThreshold";
            this.textBoxPupilThreshold.Size = new System.Drawing.Size(50, 19);
            this.textBoxPupilThreshold.TabIndex = 33;
            this.textBoxPupilThreshold.TextChanged += new System.EventHandler(this.TextBoxPupilThreshold_TextChanged);
            // 
            // textBoxReflectionThreshold
            // 
            this.textBoxReflectionThreshold.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.textBoxReflectionThreshold.BackColor = System.Drawing.SystemColors.Control;
            this.textBoxReflectionThreshold.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.textBoxReflectionThreshold.Location = new System.Drawing.Point(513, 208);
            this.textBoxReflectionThreshold.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.textBoxReflectionThreshold.Name = "textBoxReflectionThreshold";
            this.textBoxReflectionThreshold.Size = new System.Drawing.Size(50, 19);
            this.textBoxReflectionThreshold.TabIndex = 34;
            this.textBoxReflectionThreshold.TextChanged += new System.EventHandler(this.TextBoxReflectionThreshold_TextChanged);
            // 
            // EyeTrackingPipelinePupilCRQuickSettings
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.textBoxReflectionThreshold);
            this.Controls.Add(this.textBoxPupilThreshold);
            this.Controls.Add(this.trackBarReflectionThreshold);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.trackBarPupilThreshold);
            this.Controls.Add(this.label2);
            this.Margin = new System.Windows.Forms.Padding(4, 5, 4, 5);
            this.Name = "EyeTrackingPipelinePupilCRQuickSettings";
            this.Size = new System.Drawing.Size(567, 326);
            ((System.ComponentModel.ISupportInitialize)(this.trackBarPupilThreshold)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBarReflectionThreshold)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TrackBar trackBarPupilThreshold;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TrackBar trackBarReflectionThreshold;
        private System.Windows.Forms.TextBox textBoxPupilThreshold;
        private System.Windows.Forms.TextBox textBoxReflectionThreshold;
    }
}
